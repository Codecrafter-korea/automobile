"""방사형 다지 교차로 생성기 (예: 부산 연산교차로).

road_config.ROAD["legs"]에 정의된 도로들이 한 교차점(c)에 모이는
신호 교차로를 SUMO 파일로 만든다. road_builder.build()가 ROAD에 "legs"가
있으면 이 모듈의 build_radial()을 호출한다.

[한 도로(leg)의 구조]  — 교차점에서 바깥쪽을 본 그림, 우측통행

    교차점 c ◀── inn_X ──  kin_X  ◀──── in_X ──── ain_X   (진입 방향)
             ── outn_X ─▶ kout_X ───── out_X ───▶ aout_X  (진출 방향)

    · 진입/진출은 서로 다른 일방통행 엣지 (왕복 분리 도로)
    · kin_X : 우회전 연결로가 갈라지는 지점 (교차점에서 split_dist)
    · kout_X: 다른 도로에서 온 우회전 연결로가 합류하는 지점
    · slip_X: kin_X → (반시계 방향 이웃 도로의) kout_Y. 우측통행에서
              반시계 방향 이웃이 바로 "우회전" 방향이다.

[좌표계]
    교차점 = (0, 0), x = 동쪽, y = 북쪽 (m).
    방위각 b(북 = 0, 시계방향)인 도로의 바깥 방향 단위벡터 u = (sin b, cos b).
    진출 차로는 u의 오른쪽(시계방향 90°), 진입 차로는 왼쪽에 놓인다.

[생성 파일]  (env/sumo/)
    highway.nod.xml / .edg.xml / .con.xml → netconvert → highway.net.xml
    highway.rou.xml   : 차량 타입, 경로, 배경 교통 flow
    highway.tls.xml   : 신호 주기 (도로별 동시신호)
    highway.meta.json : 환경(sumo_env.py)이 쓰는 경로/진입 도로 정보
    highway.gui.xml, highway.scenery.xml, highway.sumocfg
"""
import json
import math
import os
import subprocess
import xml.etree.ElementTree as ET

from sumolib import checkBinary

from env.mdp_config import SIMULATION

LANE_W = 3.2          # 차로 폭 (m)
CENTER = "c"          # 교차점 노드 id = 신호 id


def _vec(bearing):
    """방위각 → (바깥 방향 u, 오른쪽 r) 단위벡터."""
    b = math.radians(bearing)
    u = (math.sin(b), math.cos(b))
    r = (u[1], -u[0])          # u를 시계방향으로 90° 회전
    return u, r


def _pt(u, r, along, side):
    """교차점에서 u 방향으로 along, r 방향으로 side 만큼 떨어진 점."""
    return (u[0] * along + r[0] * side, u[1] * along + r[1] * side)


def _fmt(p):
    return f"{p[0]:.2f},{p[1]:.2f}"


def build_radial(road: dict, ego: dict, traffic: dict, signal: dict,
                 out_dir: str) -> str:
    from env.road_builder import make_vtypes_block   # 순환 import 방지

    legs = road["legs"]
    ids = [lg["id"] for lg in legs]
    by_id = {lg["id"]: lg for lg in legs}
    speed = float(road["speed_limit"])
    D = float(road.get("split_dist", 60.0))
    J = float(road.get("junction_radius", 20.0))
    median = float(road.get("median", 2.0))
    slip_on = bool(road.get("slip_lanes", True))
    signal_on = bool(signal.get("enabled", True))

    # 우회전 방향 = 방위각이 바로 작은(반시계 방향) 이웃 도로
    order = sorted(ids, key=lambda i: by_id[i]["bearing"])
    right_of = {order[k]: order[k - 1] for k in range(len(order))}

    # ──────────────────────────────────────────────────────
    # 1) 노드
    # ──────────────────────────────────────────────────────
    # radius: 교차점 모서리의 회전 반경. netconvert는 이 값이 클수록 교차로
    # 면적(차량이 신호를 받고 가로지르는 구역)을 넓게 만든다.
    nodes = [f'    <node id="{CENTER}" x="0" y="0" radius="{J:g}" '
             f'type="{"traffic_light" if signal_on else "priority"}"/>']
    geo = {}
    for lg in legs:
        u, r = _vec(lg["bearing"])
        w_in = median / 2 + lg["lanes_in"] * LANE_W / 2     # 진입 차도 중심선 오프셋
        w_out = median / 2 + lg["lanes_out"] * LANE_W / 2   # 진출 차도 중심선 오프셋
        L = float(lg["length"])
        g = dict(u=u, r=r, w_in=w_in, w_out=w_out,
                 ain=_pt(u, r, L, -w_in), kin=_pt(u, r, D, -w_in),
                 aout=_pt(u, r, L, w_out), kout=_pt(u, r, D, w_out),
                 jin=_pt(u, r, J, -w_in), jout=_pt(u, r, J, w_out))
        geo[lg["id"]] = g
        X = lg["id"]
        nodes += [f'    <node id="ain_{X}" x="{g["ain"][0]:.2f}" y="{g["ain"][1]:.2f}"/>',
                  f'    <node id="kin_{X}" x="{g["kin"][0]:.2f}" y="{g["kin"][1]:.2f}" type="priority"/>',
                  f'    <node id="kout_{X}" x="{g["kout"][0]:.2f}" y="{g["kout"][1]:.2f}" type="priority"/>',
                  f'    <node id="aout_{X}" x="{g["aout"][0]:.2f}" y="{g["aout"][1]:.2f}"/>']
    with open(os.path.join(out_dir, "highway.nod.xml"), "w", encoding="utf-8") as f:
        f.write("<nodes>\n" + "\n".join(nodes) + "\n</nodes>\n")

    # ──────────────────────────────────────────────────────
    # 2) 엣지
    #    spreadType="center": 차로를 엣지 선 중심으로 좌우에 펼친다.
    #    inn/outn은 교차점 반경 J까지 차도와 평행하게 오도록 중간점을 준다.
    #    priority: 우회전 연결로(1)가 본선(3)에 합류할 때 양보하게 한다.
    # ──────────────────────────────────────────────────────
    edges = []
    for lg in legs:
        X, g = lg["id"], geo[lg["id"]]
        ni, no = lg["lanes_in"], lg["lanes_out"]
        common = f'speed="{speed}" priority="3" spreadType="center"'
        edges += [
            f'    <edge id="in_{X}" from="ain_{X}" to="kin_{X}" numLanes="{ni}" {common}/>',
            f'    <edge id="inn_{X}" from="kin_{X}" to="{CENTER}" numLanes="{ni}" {common} '
            f'shape="{_fmt(g["kin"])} {_fmt(g["jin"])} 0.00,0.00"/>',
            f'    <edge id="outn_{X}" from="{CENTER}" to="kout_{X}" numLanes="{no}" {common} '
            f'shape="0.00,0.00 {_fmt(g["jout"])} {_fmt(g["kout"])}"/>',
            f'    <edge id="out_{X}" from="kout_{X}" to="aout_{X}" numLanes="{no}" {common}/>',
        ]
        if slip_on:
            Y = right_of[X]
            a, b = g["kin"], geo[Y]["kout"]
            # 모서리 쪽으로 살짝 휘어 들어가는 곡선 (2차 베지어)
            ctrl = ((a[0] + b[0]) * 0.5 * 0.8, (a[1] + b[1]) * 0.5 * 0.8)
            pts = []
            for k in range(0, 9):
                t = k / 8
                pts.append(((1 - t) ** 2 * a[0] + 2 * (1 - t) * t * ctrl[0] + t ** 2 * b[0],
                            (1 - t) ** 2 * a[1] + 2 * (1 - t) * t * ctrl[1] + t ** 2 * b[1]))
            edges.append(
                f'    <edge id="slip_{X}" from="kin_{X}" to="kout_{Y}" numLanes="1" '
                f'speed="{min(speed, 8.33)}" priority="1" spreadType="center" '
                f'shape="{" ".join(_fmt(p) for p in pts)}"/>')
    with open(os.path.join(out_dir, "highway.edg.xml"), "w", encoding="utf-8") as f:
        f.write("<edges>\n" + "\n".join(edges) + "\n</edges>\n")

    # ──────────────────────────────────────────────────────
    # 3) 연결(connection) — 분기/합류 지점은 직접 지정, 교차점은 netconvert 추정
    #    · kin_X : 진입 차로 그대로 inn_X로, 맨 오른쪽 차로는 slip_X로도 갈 수 있음
    #    · kout_X: outn_X 차로 그대로 out_X로, 연결로는 맨 오른쪽 차로로 합류
    #    · 교차점: 연결로가 있으면 교차점 안에서의 우회전은 지운다
    # ──────────────────────────────────────────────────────
    cons = []
    for lg in legs:
        X = lg["id"]
        for l in range(lg["lanes_in"]):
            cons.append(f'    <connection from="in_{X}" to="inn_{X}" fromLane="{l}" toLane="{l}"/>')
        for l in range(lg["lanes_out"]):
            cons.append(f'    <connection from="outn_{X}" to="out_{X}" fromLane="{l}" toLane="{l}"/>')
        if slip_on:
            Y = right_of[X]
            cons.append(f'    <connection from="in_{X}" to="slip_{X}" fromLane="0" toLane="0"/>')
            cons.append(f'    <connection from="slip_{X}" to="out_{Y}" fromLane="0" toLane="0"/>')
            cons.append(f'    <delete from="inn_{X}" to="outn_{Y}"/>')
    with open(os.path.join(out_dir, "highway.con.xml"), "w", encoding="utf-8") as f:
        f.write("<connections>\n" + "\n".join(cons) + "\n</connections>\n")

    # ──────────────────────────────────────────────────────
    # 4) netconvert
    # ──────────────────────────────────────────────────────
    net_path = os.path.join(out_dir, "highway.net.xml")
    subprocess.run(
        [checkBinary("netconvert"),
         "--node-files", os.path.join(out_dir, "highway.nod.xml"),
         "--edge-files", os.path.join(out_dir, "highway.edg.xml"),
         "--connection-files", os.path.join(out_dir, "highway.con.xml"),
         "--no-turnarounds",
         # 좌표를 그대로 유지 (교차점 = 0,0). 기본값은 최소 좌표가 0이 되도록
         # 전체를 평행이동해서, GUI 시점·데이터 좌표가 어긋난다.
         "--offset.disable-normalization", "true",
         "-o", net_path],
        check=True, capture_output=True)

    # ──────────────────────────────────────────────────────
    # 5) 경로와 교통량
    #    배경차: 각 도로의 inflow를 나머지 도로로 배분 (도착 도로 진출 차로 수 비율)
    #    ego   : EGO["routes"]의 (출발, 도착) 쌍마다 ego_k 경로
    # ──────────────────────────────────────────────────────
    def route_edges(a, b):
        if slip_on and right_of[a] == b:
            return [f"in_{a}", f"slip_{a}", f"out_{b}"]
        return [f"in_{a}", f"inn_{a}", f"outn_{b}", f"out_{b}"]

    vtypes_block, flow_type = make_vtypes_block(traffic, keep_strategic=True)
    route_lines, flow_lines = [], []
    meta = {"layout": "radial", "center": CENTER, "ego_routes": [],
            "bg_routes": {}, "in_edges": {}}
    for a in ids:
        meta["in_edges"][f"in_{a}"] = {"lanes": by_id[a]["lanes_in"],
                                       "length": float(by_id[a]["length"]) - D,
                                       "inflow": float(by_id[a]["inflow"])}
        dests = [b for b in ids if b != a]
        wsum = sum(by_id[b]["lanes_out"] for b in dests)
        for b in dests:
            rate = float(by_id[a]["inflow"]) * by_id[b]["lanes_out"] / wsum
            rid = f"bg_{a}_{b}"
            route_lines.append(f'    <route id="{rid}" edges="{" ".join(route_edges(a, b))}"/>')
            flow_lines.append(
                f'    <flow id="f_{a}_{b}" type="{flow_type}" route="{rid}" begin="0" '
                f'end="100000" vehsPerHour="{rate:.1f}" departLane="best" '
                f'departSpeed="{traffic.get("depart_speed", "max")}"/>')
            meta["bg_routes"][rid] = {"first_edge": f"in_{a}", "rate": rate}
    for k, (a, b) in enumerate(ego.get("routes", [])):
        rid = f"ego_{k}"
        route_lines.append(f'    <route id="{rid}" edges="{" ".join(route_edges(a, b))}"/>')
        meta["ego_routes"].append(rid)
    with open(os.path.join(out_dir, "highway.rou.xml"), "w", encoding="utf-8") as f:
        f.write(f"""<routes>
    <vType id="ego_type" accel="{ego['accel']}" decel="{ego['decel']}" maxSpeed="{ego['max_speed']}" color="1,0,0" vClass="passenger" guiShape="passenger/sedan"/>
{vtypes_block}
{chr(10).join(route_lines)}
{chr(10).join(flow_lines)}
</routes>
""")
    with open(os.path.join(out_dir, "highway.meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)

    # ──────────────────────────────────────────────────────
    # 6) 신호 주기: legs 순서대로 도로별 동시신호
    #    [도로 X 녹색 → 황색 → 전적색] × 도로 수
    # ──────────────────────────────────────────────────────
    additional = ["highway.scenery.xml"]
    if signal_on:
        group = {}
        for c in ET.parse(net_path).getroot().iter("connection"):
            if c.get("tl") == CENTER:
                group[int(c.get("linkIndex"))] = c.get("from")[len("inn_"):]
        n = max(group) + 1
        yellow = float(signal.get("yellow", 3.0))
        all_red = float(signal.get("all_red", 2.0))
        lines = [f'    <tlLogic id="{CENTER}" type="static" programID="custom" offset="0">']
        for lg in legs:
            X = lg["id"]
            for dur, ch in ((float(lg.get("green", 20.0)), "G"), (yellow, "y"),
                            (all_red, "r")):
                if dur <= 0:
                    continue
                state = "".join(ch if group.get(i) == X else "r" for i in range(n))
                lines.append(f'        <phase duration="{dur:g}" state="{state}"/>')
        lines.append("    </tlLogic>")
        with open(os.path.join(out_dir, "highway.tls.xml"), "w") as f:
            f.write("<additional>\n" + "\n".join(lines) + "\n</additional>\n")
        additional.append("highway.tls.xml")

    # ──────────────────────────────────────────────────────
    # 7) GUI / 장식 / sumocfg
    # ──────────────────────────────────────────────────────
    extent = 2.0 * max(float(lg["length"]) for lg in legs)
    view_width = float(SIMULATION.get("gui_view_width", 350.0))
    zoom = extent / view_width * 100.0
    with open(os.path.join(out_dir, "highway.gui.xml"), "w") as f:
        f.write(f"""<viewsettings>
    <scheme name="real world"/>
    <viewport zoom="{zoom:.0f}" x="0.0" y="0.0" angle="0"/>
    <delay value="{float(SIMULATION.get("gui_delay", 100.0))}"/>
</viewsettings>
""")
    with open(os.path.join(out_dir, "highway.scenery.xml"), "w") as f:
        f.write("<additional>\n</additional>\n")
    cfg_path = os.path.join(out_dir, "highway.sumocfg")
    with open(cfg_path, "w") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="highway.net.xml"/>
        <route-files value="highway.rou.xml"/>
        <additional-files value="{",".join(additional)}"/>
    </input>
    <gui_only>
        <gui-settings-file value="highway.gui.xml"/>
    </gui_only>
</configuration>
""")
    return cfg_path
