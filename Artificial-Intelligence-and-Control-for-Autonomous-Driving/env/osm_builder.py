"""OpenStreetMap 데이터로 교차로 주변 도로망 전체를 만드는 생성기.

road_config.ROAD["osm"]["enabled"]가 True면 road_builder.build()가
이 모듈의 build_osm()을 호출한다.

[흐름]
    env/osm/*.osm (OpenStreetMap에서 받은 원본, 저장소에 포함)
        │  netconvert : 도로·차로·연결·신호 → highway.net.xml
        │  polyconvert: 건물 → highway.poly.xml (GUI 배경, 시뮬레이션과 무관)
        ▼
    도로 끝점(fringe) 탐색
        · ROAD["legs"]의 6개 간선: 도로 이름 + 교차점 기준 방위각으로 찾음
        · 나머지 끝점: 골목길 입구 (작은 교통량)
        ▼
    경로(최단경로)와 flow 생성 → highway.rou.xml
    환경용 정보 → highway.meta.json

[OSM 데이터 갱신 방법]
    Overpass API(https://overpass-api.de)에서 같은 범위를 다시 받아
    env/osm/yeonsan.osm을 덮어쓰면 된다. 받는 범위는 ROAD["osm"]["bbox"].
"""
import json
import math
import os
import subprocess
import xml.etree.ElementTree as ET

from sumolib import checkBinary
import sumolib

from env.mdp_config import SIMULATION

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# OSM 도로 등급별 기본값 (OSM에 차로 수/제한속도 태그가 없을 때만 쓰임).
# 도심 안전속도 5030: 간선 50 km/h, 이면도로 30 km/h.
TYPES_XML = """<types>
  <type id="highway.trunk"          priority="13" numLanes="2" speed="16.67" oneway="false"/>
  <type id="highway.trunk_link"     priority="12" numLanes="1" speed="8.33"  oneway="true"/>
  <type id="highway.primary"        priority="12" numLanes="2" speed="13.89" oneway="false"/>
  <type id="highway.primary_link"   priority="11" numLanes="1" speed="8.33"  oneway="true"/>
  <type id="highway.secondary"      priority="11" numLanes="2" speed="13.89" oneway="false"/>
  <type id="highway.secondary_link" priority="10" numLanes="1" speed="8.33"  oneway="true"/>
  <type id="highway.tertiary"       priority="8"  numLanes="1" speed="13.89" oneway="false"/>
  <type id="highway.tertiary_link"  priority="7"  numLanes="1" speed="8.33"  oneway="true"/>
  <type id="highway.unclassified"   priority="5"  numLanes="1" speed="8.33"  oneway="false"/>
  <type id="highway.residential"    priority="4"  numLanes="1" speed="8.33"  oneway="false"/>
  <type id="highway.living_street"  priority="3"  numLanes="1" speed="5.56"  oneway="false"/>
</types>
"""


def _bearing(dx, dy):
    return math.degrees(math.atan2(dx, dy)) % 360.0


def _angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def build_osm(road: dict, ego: dict, traffic: dict, signal: dict,
              out_dir: str) -> str:
    from env.road_builder import make_vtypes_block   # 순환 import 방지

    cfg = road["osm"]
    osm_file = os.path.join(BASE, cfg["file"])
    center_id = str(cfg["center_node"])
    net_path = os.path.join(out_dir, "highway.net.xml")
    types_path = os.path.join(out_dir, "osm_types.typ.xml")
    with open(types_path, "w") as f:
        f.write(TYPES_XML)

    # ──────────────────────────────────────────────────────
    # 1) netconvert: OSM → SUMO 도로망
    #    --tls.guess-signals : OSM 신호 노드를 가까운 교차로 신호로 합침
    #    --tls.set           : OSM에 신호 표시가 없는 연산교차로 중심을 신호로 지정
    #    --keep-edges.in-geo-boundary : bbox에 걸친 도로만 남김
    # ──────────────────────────────────────────────────────
    minlon, minlat, maxlon, maxlat = cfg["bbox"]
    subprocess.run(
        [checkBinary("netconvert"),
         "--osm-files", osm_file, "--type-files", types_path,
         "--geometry.remove", "--ramps.guess", "--junctions.join",
         "--tls.guess-signals", "--tls.join", "--tls.default-type", "static",
         "--tls.set", center_id,
         "--tls.yellow.time", str(int(round(signal.get("yellow", 3.0)))),  # 정수만 허용
         "--remove-edges.isolated", "--keep-edges.by-vclass", "passenger",
         "--no-turnarounds.except-deadend", "--output.street-names",
         "--keep-edges.in-geo-boundary", f"{minlon},{minlat},{maxlon},{maxlat}",
         "-o", net_path],
        check=True, capture_output=True)

    # 건물 (GUI 배경). 실패해도 시뮬레이션에는 영향 없음.
    additional = []
    if cfg.get("buildings", True):
        poly_path = os.path.join(out_dir, "highway.poly.xml")
        typ = os.path.join(os.environ["SUMO_HOME"], "data", "typemap",
                           "osmPolyconvert.typ.xml")
        res = subprocess.run(
            [checkBinary("polyconvert"), "--osm-files", osm_file,
             "--net-file", net_path, "--type-file", typ, "-o", poly_path],
            capture_output=True)
        if res.returncode == 0:
            additional.append("highway.poly.xml")

    net = sumolib.net.readNet(net_path)
    center = net.getNode(center_id)
    cx, cy = center.getCoord()

    # ──────────────────────────────────────────────────────
    # 2) 도로 끝점(fringe) 찾기
    #    끝점 = 이웃 노드가 하나뿐이거나 막다른(dead_end) 노드 (도로망 바깥 경계)
    #    진입 엣지 = 끝점에서 나가는 엣지, 진출 엣지 = 끝점으로 들어오는 엣지
    # ──────────────────────────────────────────────────────
    entries, exits = [], []      # (edge, 방위각, 거리, 도로 이름)
    for nd in net.getNodes():
        nbrs = ({e.getFromNode() for e in nd.getIncoming()}
                | {e.getToNode() for e in nd.getOutgoing()})
        if len(nbrs) != 1 and nd.getType() != "dead_end":
            continue
        x, y = nd.getCoord()
        b, d = _bearing(x - cx, y - cy), math.hypot(x - cx, y - cy)
        for e in nd.getOutgoing():
            if e.allows("passenger"):
                entries.append((e, b, d, e.getName()))
        for e in nd.getIncoming():
            if e.allows("passenger"):
                exits.append((e, b, d, e.getName()))

    def heading(e, at_end):
        """엣지의 끝(또는 시작) 부분 진행 방향 (방위각)."""
        s = e.getShape()
        (x0, y0), (x1, y1) = (s[-2], s[-1]) if at_end else (s[0], s[1])
        return _bearing(x1 - x0, y1 - y0)

    def trace_leg(lg, outgoing):
        """교차점에서 간선을 따라 바깥으로 이어 가서 도로망 끝 엣지를 찾는다.

        outgoing=True : 교차점에서 나가는 차도를 따라감 → 진출 끝 엣지
        outgoing=False: 교차점으로 들어오는 차도를 거슬러 감 → 진입 시작 엣지
        같은 도로 이름이면서 가장 덜 꺾이는 다음 엣지를 고른다.
        """
        def far_point(e):
            s = e.getShape()
            return s[-1] if outgoing else s[0]
        first = center.getOutgoing() if outgoing else center.getIncoming()
        cands = [e for e in first if e.allows("passenger")
                 and _angle_diff(_bearing(far_point(e)[0] - cx, far_point(e)[1] - cy),
                                 lg["bearing"]) < 40.0]
        cands = [e for e in cands if lg["road"] in (e.getName() or "")] or cands
        if not cands:
            raise RuntimeError(f"OSM 도로망에서 '{lg['road']}' ({lg['bearing']}°) "
                               f"방향 도로를 교차점에서 찾지 못했습니다.")
        e = min(cands, key=lambda e: _angle_diff(
            _bearing(far_point(e)[0] - cx, far_point(e)[1] - cy), lg["bearing"]))
        seen = {e.getID()}
        for _ in range(200):
            nxt = list(e.getOutgoing().keys()) if outgoing else list(e.getIncoming().keys())
            nxt = [n for n in nxt if n.getID() not in seen and n.allows("passenger")
                   and n.getName() == e.getName()
                   and n.getID().lstrip("-") != e.getID().lstrip("-")]   # 유턴 제외
            if not nxt:
                return e
            h = heading(e, at_end=outgoing)
            e = min(nxt, key=lambda n: _angle_diff(heading(n, at_end=not outgoing), h))
            seen.add(e.getID())
        return e

    legs = road["legs"]
    leg_in = {lg["id"]: trace_leg(lg, outgoing=False) for lg in legs}
    leg_out = {lg["id"]: trace_leg(lg, outgoing=True) for lg in legs}
    used = {e.getID() for e in list(leg_in.values()) + list(leg_out.values())}
    side_in = [c[0] for c in entries if c[0].getID() not in used]
    side_out = [c[0] for c in exits if c[0].getID() not in used]

    # ──────────────────────────────────────────────────────
    # 3) 교통 수요
    #    · 간선 → 간선: 도로별 inflow를 다른 간선으로 배분 (진출 차로 수 비율)
    #      + 그중 side_share만큼은 골목길로 빠짐
    #    · 골목길 → 간선: 입구마다 side_inflow 대/시간
    #    경로는 sumolib 최단경로 (실제 연결/회전 금지를 따름)
    # ──────────────────────────────────────────────────────
    path_cache = {}

    def path(a, b):
        key = (a.getID(), b.getID())
        if key not in path_cache:
            p, _ = net.getShortestPath(a, b, vClass="passenger")
            path_cache[key] = [e.getID() for e in p] if p else None
        return path_cache[key]

    vtypes_block, flow_type = make_vtypes_block(traffic, keep_strategic=True)
    route_lines, flow_lines = [], []
    meta = {"layout": "osm", "center": center_id, "center_xy": [cx, cy],
            "ego_routes": [], "ego_depart_pos": {}, "bg_routes": {}, "in_edges": {}}
    side_share = float(cfg.get("side_share", 0.1))
    side_inflow = float(cfg.get("side_inflow", 40.0))
    k = 0

    def add_flow(src, dst, rate):
        nonlocal k
        p = path(src, dst)
        if p is None or rate <= 0:
            return
        rid = f"bg_{k}"
        k += 1
        route_lines.append(f'    <route id="{rid}" edges="{" ".join(p)}"/>')
        flow_lines.append(
            f'    <flow id="f_{rid}" type="{flow_type}" route="{rid}" begin="0" '
            f'end="100000" vehsPerHour="{rate:.1f}" departLane="best" '
            f'departSpeed="{traffic.get("depart_speed", "max")}"/>')
        meta["bg_routes"][rid] = {"first_edge": src.getID(), "rate": rate}
        meta["in_edges"].setdefault(src.getID(), {
            "lanes": src.getLaneNumber(), "length": src.getLength(), "inflow": 0.0})
        meta["in_edges"][src.getID()]["inflow"] += rate

    by_id = {lg["id"]: lg for lg in legs}
    for a in by_id:
        dests = [b for b in by_id if b != a]
        wsum = sum(by_id[b]["lanes_out"] for b in dests)
        main = float(by_id[a]["inflow"]) * (1.0 - side_share)
        for b in dests:
            add_flow(leg_in[a], leg_out[b], main * by_id[b]["lanes_out"] / wsum)
        if side_out:
            side_rate = float(by_id[a]["inflow"]) * side_share / len(side_out)
            for s in side_out:
                add_flow(leg_in[a], s, side_rate)
    for s in side_in:
        for b in by_id:
            add_flow(s, leg_out[b], side_inflow / len(by_id))

    # ego 경로: 출발 간선 끝점 → 도착 간선 끝점. 출발 위치는 교차점에서
    # ego_approach(m) 떨어진 지점 (긴 도로 끝에서 출발하면 에피소드가 너무 길어짐)
    approach = float(cfg.get("ego_approach", 400.0))
    for i, (a, b) in enumerate(ego.get("routes", [])):
        p = path(leg_in[a], leg_out[b])
        if p is None:
            raise RuntimeError(f"ego 경로 {a}→{b}를 만들 수 없습니다.")
        rid = f"ego_{i}"
        route_lines.append(f'    <route id="{rid}" edges="{" ".join(p)}"/>')
        meta["ego_routes"].append(rid)
        # 경로상 교차점까지 거리 → 첫 엣지 위 출발 위치
        to_center, edges = 0.0, [net.getEdge(e) for e in p]
        for e in edges:
            to_center += e.getLength()
            if e.getToNode().getID() == center_id or center_id in e.getToNode().getID():
                break
        first_len = edges[0].getLength()
        meta["ego_depart_pos"][rid] = float(
            min(max(to_center - approach, 0.0), max(first_len - 10.0, 0.0)))

    with open(os.path.join(out_dir, "highway.rou.xml"), "w", encoding="utf-8") as f:
        f.write(f"""<routes>
    <vType id="ego_type" accel="{ego['accel']}" decel="{ego['decel']}" maxSpeed="{ego['max_speed']}" color="1,0,0" vClass="passenger" guiShape="passenger/sedan"/>
{vtypes_block}
{chr(10).join(route_lines)}
{chr(10).join(flow_lines)}
</routes>
""")

    # ──────────────────────────────────────────────────────
    # 4) 중심 교차로 신호: 도로별 동시신호로 교체
    #    netconvert가 만든 신호(중심 + 바로 옆 OSM 신호 지점이 합쳐진 것)의
    #    연결마다 "어느 간선에서 들어온 차량인가"를 출발 엣지 위치로 판정해
    #    legs 순서대로 녹색을 준다.
    # ──────────────────────────────────────────────────────
    root = ET.parse(net_path).getroot()
    center_tl = None
    for c in root.iter("connection"):
        if c.get("tl") and net.getEdge(c.get("from")).getToNode().getID() == center_id:
            center_tl = c.get("tl")
            break
    if center_tl and signal.get("enabled", True):
        group = {}
        for c in root.iter("connection"):
            if c.get("tl") != center_tl:
                continue
            e = net.getEdge(c.get("from"))
            # 엣지 시작점(교차점에서 먼 쪽)의 방위각으로 간선 판정
            sx, sy = e.getShape()[0]
            b = _bearing(sx - cx, sy - cy)
            # 교차점에서 멀어지는 방향의 엣지(진출 차도)는 신호를 받을 필요가 없음
            ex, ey = e.getShape()[-1]
            outward = math.hypot(ex - cx, ey - cy) > math.hypot(sx - cx, sy - cy) + 1.0
            leg = min(legs, key=lambda lg: _angle_diff(b, lg["bearing"]))["id"]
            group[int(c.get("linkIndex"))] = None if outward else leg
        n = max(group) + 1
        yellow = float(signal.get("yellow", 3.0))
        all_red = float(signal.get("all_red", 2.0))
        lines = [f'    <tlLogic id="{center_tl}" type="static" programID="custom" offset="0">']
        for lg in legs:
            for dur, ch in ((float(lg.get("green", 20.0)), "G"), (yellow, "y"),
                            (all_red, "r")):
                if dur <= 0:
                    continue
                state = "".join(
                    "G" if group.get(i) is None else (ch if group.get(i) == lg["id"] else "r")
                    for i in range(n))
                lines.append(f'        <phase duration="{dur:g}" state="{state}"/>')
        lines.append("    </tlLogic>")
        with open(os.path.join(out_dir, "highway.tls.xml"), "w") as f:
            f.write("<additional>\n" + "\n".join(lines) + "\n</additional>\n")
        additional.append("highway.tls.xml")
    meta["center_tls"] = center_tl
    with open(os.path.join(out_dir, "highway.meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)

    # ──────────────────────────────────────────────────────
    # 5) GUI / sumocfg
    # ──────────────────────────────────────────────────────
    xmin, ymin, xmax, ymax = net.getBoundary()
    extent = max(xmax - xmin, ymax - ymin)
    view_width = float(SIMULATION.get("gui_view_width", 350.0))
    with open(os.path.join(out_dir, "highway.gui.xml"), "w") as f:
        f.write(f"""<viewsettings>
    <scheme name="real world"/>
    <viewport zoom="{extent / view_width * 100.0:.0f}" x="{cx:.1f}" y="{cy:.1f}" angle="0"/>
    <delay value="{float(SIMULATION.get("gui_delay", 100.0))}"/>
</viewsettings>
""")
    with open(os.path.join(out_dir, "highway.scenery.xml"), "w") as f:
        f.write("<additional>\n</additional>\n")
    additional.insert(0, "highway.scenery.xml")
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
