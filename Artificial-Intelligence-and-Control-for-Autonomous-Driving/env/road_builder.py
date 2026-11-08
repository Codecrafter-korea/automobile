"""configs/road_config.py 를 읽어 SUMO 도로 파일들을 자동 생성하는 모듈.

SUMO는 도로를 여러 XML 파일로 정의한다:
    highway.nod.xml  : 노드(교차점/도로 끝점)의 좌표
    highway.edg.xml  : 엣지(도로 구간) — 어느 노드에서 어느 노드로, 차선 수, 제한속도
    highway.net.xml  : 위 두 파일을 netconvert 도구로 컴파일한 "완성된 도로망"
                       (SUMO가 실제로 읽는 파일. 직접 작성하기엔 복잡해서 도구로 생성)
    highway.rou.xml  : 차량 타입(vType)과 교통 수요(flow/route) 정의
    highway.sumocfg  : 위 파일들을 묶어주는 설정 파일 (시뮬레이션 실행의 진입점)

train.py / test.py 가 시작할 때마다 build()를 호출하므로,
road_config.py만 수정하면 도로가 자동으로 다시 만들어진다.

[도로 구조: 사거리 교차로]
기존에는 노드 2개 + 엣지 1개짜리 단순 직선 도로였으나,
동서 도로 중앙에 남북 도로가 교차하는 사거리 구조로 확장했다.

SUMO에서 교차로는 "노드 위에만" 생길 수 있다. 도로 중간에 다른 길을
꽂아 넣을 수 없으므로, 기존 직선 e0 하나를 교차점에서 둘로 쪼갠다:

                      nn (북)
                       │ c0
                       ↓
    n0 ──── e0 ────▶  nc  ──── e1 ────▶ n1
                       │ c1
                       ↓
                      ns (남)

    n0  : 동서 도로 출발점 (ego 시작점)
    nc  : 교차점
    n1  : 동서 도로 도착점
    nn  : 남북 도로 시작점(북)
    ns  : 남북 도로 도착점(남)

엣지 연결(어느 차선에서 어느 차선으로 갈 수 있는지)은 netconvert가
자동으로 생성해준다. 교차점 nc는 road_config.SIGNAL["enabled"]에 따라
신호 교차로(type="traffic_light") 또는 우선권 교차로(type="priority")가 된다.

※ 첫 구간의 id를 "e0"로 그대로 유지한 것은 의도적이다.
  sumo_env.py가 ego를 투입할 때 e0를 기준으로 삼기 때문에,
  이름을 바꾸면 ego가 생성되지 않는다.
"""
import os
import subprocess

from sumolib import checkBinary  # SUMO 실행파일(sumo, netconvert 등) 경로를 찾아주는 유틸
from env.mdp_config import SIMULATION
try:
    from env.road_config import SCENERY
except ImportError:      # 구버전 설정 하위호환
    SCENERY = {"trees": False}
try:
    from env.road_config import SIGNAL
except ImportError:      # 신호 설정이 없으면 신호 없는 교차로
    SIGNAL = {"enabled": False}

TLS_ID = "nc"            # 신호 교차로 id (= 교차점 노드 id)


def build(road: dict, ego: dict, traffic: dict, out_dir: str) -> str:
    """SUMO 파일들을 out_dir에 생성하고, sumocfg 파일 경로를 반환한다.

    Args:
        road:    configs.road_config.ROAD    (도로 형상)
        ego:     configs.road_config.EGO     (RL 차량 속성)
        traffic: configs.road_config.TRAFFIC (배경 교통류)
        out_dir: 생성 위치 (보통 프로젝트의 sumo/ 폴더)
    """
    os.makedirs(out_dir, exist_ok=True)

    # OpenStreetMap 기반 주변 도로망 전체 (예: 연산교차로 반경 약 500m)
    if (road.get("osm") or {}).get("enabled"):
        from env.osm_builder import build_osm
        return build_osm(road, ego, traffic, SIGNAL, out_dir)

    # 방사형 다지 교차로(예: 부산 연산교차로)는 별도 빌더가 만든다.
    if "legs" in road:
        from env.junction_builder import build_radial
        return build_radial(road, ego, traffic, SIGNAL, out_dir)

    # ──────────────────────────────────────────────────────
    # 0) 교차로 형상 파라미터
    #    road_config.py에 값이 있으면 그걸 쓰고, 없으면 기본값으로 동작한다
    #    (기존 road_config.py를 수정하지 않아도 그대로 돌아가도록).
    #
    #    cross_x    : 교차점의 x 좌표. 기본값은 도로 중앙.
    #    cross_arm  : 교차점에서 남북 도로 끝까지의 길이(m).
    #    cross_lanes/cross_speed : 남북 도로의 차선 수와 제한속도.
    #                 간선도로(동서)보다 작게 주어 "주도로 + 종도로" 느낌을 낸다.
    # ──────────────────────────────────────────────────────
    cross_x = float(road.get("cross_x", road["length"] / 2.0))
    cross_arm = float(road.get("cross_arm", 2000.0))
    cross_lanes = int(road.get("cross_lanes", 2))
    cross_speed = float(road.get("cross_speed", 16.67))   # ≈ 60 km/h

    # ──────────────────────────────────────────────────────
    # 1) 노드 파일: 사거리를 구성하는 5개 노드
    #    n0 = 원점(출발), n1 = length만큼 떨어진 지점(도착)
    #    nc = 교차점,  nn = 북쪽 끝,  ns = 남쪽 끝
    #
    #    SIGNAL["enabled"]가 True면 type="traffic_light"(신호 교차로),
    #    False면 type="priority"(신호 없이 우선권/양보로 운영)로 만든다.
    #    신호 주기는 아래 7)에서 SIGNAL 설정으로 따로 생성한다.
    # ──────────────────────────────────────────────────────
    signal_on = bool(SIGNAL.get("enabled", False))
    nc_type = "traffic_light" if signal_on else "priority"
    with open(os.path.join(out_dir, "highway.nod.xml"), "w") as f:
        f.write(f"""<nodes>
    <node id="n0" x="0" y="0"/>
    <node id="{TLS_ID}" x="{cross_x}" y="0" type="{nc_type}"/>
    <node id="n1" x="{road['length']}" y="0"/>
    <node id="nn" x="{cross_x}" y="{cross_arm}"/>
    <node id="ns" x="{cross_x}" y="-{cross_arm}"/>
</nodes>
""")

    # ──────────────────────────────────────────────────────
    # 2) 엣지 파일: 동서 2구간 + 남북 2구간
    #    numLanes = 차선 수, speed = 제한속도(m/s)
    #
    #    e0 : 출발점 → 교차점   (ego가 시작하는 구간. id 유지 필수)
    #    e1 : 교차점 → 도착점
    #    c0 : 북 → 교차점       (남북 방향 교차 교통 진입)
    #    c1 : 교차점 → 남       (남북 방향 교차 교통 진출)
    # ──────────────────────────────────────────────────────
    with open(os.path.join(out_dir, "highway.edg.xml"), "w") as f:
        f.write(f"""<edges>
    <edge id="e0" from="n0" to="nc" numLanes="{road['num_lanes']}" speed="{road['speed_limit']}"/>
    <edge id="e1" from="nc" to="n1" numLanes="{road['num_lanes']}" speed="{road['speed_limit']}"/>
    <edge id="c0" from="nn" to="nc" numLanes="{cross_lanes}" speed="{cross_speed}"/>
    <edge id="c1" from="nc" to="ns" numLanes="{cross_lanes}" speed="{cross_speed}"/>
</edges>
""")

    # ──────────────────────────────────────────────────────
    # 3) 라우트 파일: 차량 타입 2종 + 배경 교통 flow
    #
    #    <vType>  : 차량의 물리 속성 템플릿
    #       ego_type : RL이 조종할 차 (빨간색, color="1,0,0" = RGB)
    #    <vTypeDistribution> : 배경차 컨트롤러 믹스.
    #       TRAFFIC["controllers"]에 정의된 여러 차량추종모델(Krauss/IDM/
    #       EIDM/ACC 등)을 probability 비율로 섞는다. flow가 이 분포를
    #       type으로 쓰면, 투입되는 차량마다 SUMO가 확률적으로 타입을 뽑는다.
    #    <route>  : 지나갈 엣지들의 나열.
    #       r0 = e0 e1 : 동서 직진 (교차로를 통과해야 하므로 두 구간을 모두 적는다.
    #                    e0만 적으면 차량이 교차로에서 사라진다. ego도 이 경로를 쓴다)
    #       r1 = c0 c1 : 남북 직진 (교차 교통)
    #    <flow>   : 교통 수요 정의. period="X"는 "X초마다 정확히 1대씩"
    #               → 등간격(uniform) 생성. (vehsPerHour와 등가지만
    #               "비슷한 간격으로 새로 생성"을 명시적으로 보장)
    #               교차 방향 flow(cross)는 교차로에서 실제 상호작용(양보/대기)이
    #               일어나도록 추가한 것이며, 교통량은 동서 도로와 동일하게 둔다.
    #               ego는 여기에 없고, 환경(sumo_env.py)이 reset마다
    #               traci.vehicle.add()로 직접 투입한다.
    # ──────────────────────────────────────────────────────
    vtypes_block, flow_type = make_vtypes_block(traffic)

    period = 3600.0 / traffic['vehs_per_hour']

    with open(os.path.join(out_dir, "highway.rou.xml"), "w") as f:
        f.write(f"""<routes>
    <vType id="ego_type" accel="{ego['accel']}" decel="{ego['decel']}" maxSpeed="{ego['max_speed']}" color="1,0,0" vClass="passenger" guiShape="passenger/sedan"/>
{vtypes_block}
    <route id="r0" edges="e0 e1"/>
    <route id="r1" edges="c0 c1"/>
    <flow id="traffic" type="{flow_type}" route="r0" begin="0" end="100000"
          period="{period:.2f}"
          departLane="{traffic['depart_lane']}" departSpeed="{traffic['depart_speed']}"/>
    <flow id="cross" type="{flow_type}" route="r1" begin="0" end="100000"
          period="{period:.2f}"
          departLane="random" departSpeed="{traffic['depart_speed']}"/>
</routes>
""")
    return _finish_straight(road, ego, traffic, out_dir, cross_x, signal_on)


def make_vtypes_block(traffic: dict, keep_strategic: bool = False):
    """TRAFFIC["controllers"]로 배경차 vTypeDistribution XML을 만든다.

    Returns: (XML 문자열, flow에서 쓸 type id)
    keep_strategic=True면 keep_lane이어도 "경로상 꼭 필요한 차선변경"
    (lcStrategic)은 끄지 않는다. 회전 경로가 있는 교차로에서는 회전 차선으로
    미리 옮겨가야 하므로 반드시 True로 써야 한다.
    """
    controllers = traffic.get("controllers")
    if controllers:
        # probability 합이 1이 아니어도 되도록 자동 정규화
        total_p = sum(float(c.get("probability", 1.0))
                      for c in controllers.values())
        vtype_lines = []
        for name, c in controllers.items():
            prob = float(c.get("probability", 1.0)) / total_p
            attrs = [
                f'id="car_{name}"',
                f'probability="{prob:.4f}"',
                f'carFollowModel="{c["carFollowModel"]}"',
                f'accel="{c.get("accel", 2.5)}"',
                f'decel="{c.get("decel", 4.5)}"',
                f'tau="{c.get("tau", 1.0)}"',
                f'minGap="{c.get("min_gap", 2.5)}"',
                # 컨트롤러별 희망속도 오버라이드 (없으면 공통 max_speed).
                # 컨트롤러마다 속도를 다르게 주면 차선 간 속도차가 생겨
                # "추월할 이유"가 자연스럽게 만들어진다.
                f'maxSpeed="{c.get("max_speed", traffic["max_speed"])}"',
                # speedDev: 같은 타입 안에서도 개체별 희망속도 편차 (0~)
                f'speedDev="{c.get("speed_dev", 0.1)}"',
                f'color="{c.get("color", "1,1,0")}"',
                'vClass="passenger"', 'guiShape="passenger/sedan"',
            ]
            # sigma는 Krauss 계열에만 의미가 있으므로 지정된 경우에만 넣는다
            if "sigma" in c:
                attrs.append(f'sigma="{c["sigma"]}"')
            # ── 배경차 차선 유지 (keep_lane, 기본 True) ──
            # 배경차의 자율 차선변경(전략/협조/속도이득/우측유지)을 전부 꺼서
            # 각자 자기 차선을 지키게 한다. 이유:
            #   배경차가 자유롭게 돌아다니면 (측정: 150스텝에 477회 변경)
            #   차선 간 틈이 수시로 닫혀서, ego가 틈을 노려 파고드는
            #   "칼치기 추월"을 학습할 안정적인 기회 자체가 사라진다.
            #   배경차가 차선을 유지하면 "느린 차 뒤 → 옆 틈으로 변경 →
            #   가속"이라는 추월 패턴이 재현 가능한 형태로 나타난다.
            # 특정 컨트롤러만 돌아다니게 하려면 그 항목에 "keep_lane": False.
            #
            # 주의: lcStrategic="0"은 "경로상 꼭 필요한 차선변경"까지 끄는
            # 설정이다. 지금 두 경로(r0, r1) 모두 직진이라 문제가 없지만,
            # 좌/우회전 경로를 추가한다면 그 차량들은 회전 차선으로 미리
            # 옮겨가야 하므로 해당 컨트롤러에 "keep_lane": False를 줘야 한다.
            if c.get("keep_lane", True):
                if not keep_strategic:
                    attrs.append('lcStrategic="0"')
                attrs += ['lcCooperative="0"',
                          'lcSpeedGain="0"', 'lcKeepRight="0"']
            vtype_lines.append("        <vType " + " ".join(attrs) + "/>")
        vtypes_block = ('    <vTypeDistribution id="carDist">\n'
                        + "\n".join(vtype_lines)
                        + "\n    </vTypeDistribution>")
        flow_type = "carDist"
    else:
        # controllers 설정이 없으면 예전 방식(단일 Krauss)으로 동작 (하위호환)
        vtypes_block = (f'    <vType id="car" accel="2.5" decel="4.5" '
                        f'maxSpeed="{traffic["max_speed"]}" '
                        f'sigma="{traffic.get("sigma", 0.5)}" '
                        f'vClass="passenger" guiShape="passenger/sedan"/>')
        flow_type = "car"
    return vtypes_block, flow_type


def _finish_straight(road, ego, traffic, out_dir, cross_x, signal_on):
    """직선/사거리 예제의 GUI·장식·sumocfg·netconvert 단계."""
    # 방사형 교차로에서 남은 경로 정보 파일이 있으면 지운다
    # (남아 있으면 sumo_env가 방사형 교차로로 착각한다).
    stale = os.path.join(out_dir, "highway.meta.json")
    if os.path.exists(stale):
        os.remove(stale)
    # ──────────────────────────────────────────────────────
    # 4) GUI 기본 시각화 설정
    #    GUI 관련 값의 source of truth는 env/mdp_config.py의 SIMULATION 하나뿐이다.
    #    view_road.py / train.py / test.py / 데이터 수집 모두 build()를 거치므로
    #    gui_zoom/gui_delay를 바꾸면 생성되는 highway.gui.xml에도 즉시 반영된다.
    # ──────────────────────────────────────────────────────
    # GUI 시작 화면: "교차로"가 화면 중앙에 오도록 자동 확대해둔다.
    #   (12km 전체 맞춤으로 시작하면 도로가 가느다란 선으로만 보여 직관성이
    #    떨어진다. 사거리 구조에서는 교차로가 이 도로의 핵심이므로
    #    교차점을 화면 중앙에 두고 확대해 대기 화면으로 삼는다.
    #    ▶ 재생 후에는 env가 ego를 추적(trackVehicle)하며 시점을 가져간다.)
    # 좌표/배율은 도로 길이·차선 수에서 "계산"하므로 설정을 바꿔도 안 어긋난다:
    #   zoom(%) = 도로길이 / 보고싶은폭(gui_view_width) × 100
    #     (SUMO zoom은 "네트워크 전체 = 100%" 기준)
    gui_delay = float(SIMULATION.get("gui_delay", 100.0))
    view_width = float(SIMULATION.get("gui_view_width", 500.0))   # 시작 화면 시야(m)
    zoom = float(road["length"]) / view_width * 100.0
    center_x = cross_x          # 교차점을 화면 중앙에
    center_y = 0.0
    gui_settings_path = os.path.join(out_dir, "highway.gui.xml")
    with open(gui_settings_path, "w") as f:
        f.write(f"""<viewsettings>
    <scheme name="real world"/>
    <viewport zoom="{zoom:.0f}" x="{center_x:.1f}" y="{center_y:.1f}" angle="0"/>
    <delay value="{gui_delay}"/>
</viewsettings>
""")

    # ──────────────────────────────────────────────────────
    # 4.7) 도로변 장식(나무) — 순수 시각 요소
    #
    # SUMO의 <poly>(다각형)로 "초록 수관 + 갈색 줄기"를 그려 도로
    # 양옆에 심는다. additional 파일로 로드되며 차량 물리와 완전히
    # 무관하다 (충돌/관측/보상 어디에도 안 잡힘).
    # 좌표계: 차선은 y ∈ [0, -3.2×차선수] 영역 → 왼쪽 숲은 y > 0,
    # 오른쪽 숲은 y < -3.2×차선수 - side_offset 에 배치.
    # ──────────────────────────────────────────────────────
    _write_scenery(out_dir, road, cross_x)

    # ──────────────────────────────────────────────────────
    # 5) sumocfg: "이 net 파일과 이 route 파일로 시뮬레이션해라"
    # ──────────────────────────────────────────────────────
    #    신호가 켜져 있으면 신호 주기 파일(highway.tls.xml)도 함께 읽는다.
    # ──────────────────────────────────────────────────────
    additional = "highway.scenery.xml"
    if signal_on:
        additional += ",highway.tls.xml"
    cfg_path = os.path.join(out_dir, "highway.sumocfg")
    with open(cfg_path, "w") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="highway.net.xml"/>
        <route-files value="highway.rou.xml"/>
        <additional-files value="{additional}"/>
    </input>
    <gui_only>
        <gui-settings-file value="highway.gui.xml"/>
    </gui_only>
</configuration>
""")

    # ──────────────────────────────────────────────────────
    # 6) netconvert 실행: nod + edg → net.xml 컴파일
    #    (SUMO 설치 시 함께 깔리는 커맨드라인 도구)
    #
    #    --no-turnarounds : 교차로에서 유턴 연결을 만들지 않는다.
    #                       (유턴이 생기면 차량이 제자리에서 돌아나가며
    #                        흐름이 부자연스러워진다)
    #    신호 교차로의 기본 주기(netconvert가 자동 생성)는 아래 7)의
    #    사용자 정의 주기로 덮어쓴다.
    # ──────────────────────────────────────────────────────
    netconvert = checkBinary("netconvert")
    net_path = os.path.join(out_dir, "highway.net.xml")
    subprocess.run(
        [netconvert,
         "--node-files", os.path.join(out_dir, "highway.nod.xml"),
         "--edge-files", os.path.join(out_dir, "highway.edg.xml"),
         "--no-turnarounds",
         "-o", net_path],
        check=True,           # 실패하면 예외를 던져서 바로 알 수 있게
        capture_output=True,  # 출력은 조용히 삼킴
    )

    # ──────────────────────────────────────────────────────
    # 7) 신호 주기 파일 (SIGNAL["enabled"]일 때만)
    #    netconvert가 만든 기본 주기는 양방향 녹색 시간이 같다.
    #    SIGNAL 설정대로 주기를 새로 만들어 additional 파일로 넣으면
    #    SUMO는 "마지막에 읽은 프로그램"을 사용하므로 이 주기가 적용된다.
    # ──────────────────────────────────────────────────────
    if signal_on:
        _write_tls_program(net_path, os.path.join(out_dir, "highway.tls.xml"))
    return cfg_path


def _write_tls_program(net_path: str, out_path: str):
    """SIGNAL 설정으로 교차로 nc의 고정 신호 주기를 만든다.

    신호 상태 문자열은 "연결(link) 하나당 글자 하나"다. 어떤 linkIndex가
    어느 진입로(e0 = 동서, c0 = 남북)에 속하는지는 netconvert가 정하므로,
    완성된 net.xml의 <connection tl="nc" linkIndex=...>를 읽어 매핑한다.
    그래서 차선 수를 바꿔도 상태 문자열 길이가 자동으로 맞는다.
    """
    import xml.etree.ElementTree as ET
    group = {}                       # linkIndex -> "main" | "cross"
    for c in ET.parse(net_path).getroot().iter("connection"):
        if c.get("tl") == TLS_ID:
            group[int(c.get("linkIndex"))] = (
                "main" if c.get("from") == "e0" else "cross")
    n = max(group) + 1

    def state(main_ch, cross_ch):
        return "".join(main_ch if group.get(i) == "main" else cross_ch
                       for i in range(n))

    g_main = float(SIGNAL.get("main_green", 30.0))
    g_cross = float(SIGNAL.get("cross_green", 20.0))
    yellow = float(SIGNAL.get("yellow", 4.0))
    all_red = float(SIGNAL.get("all_red", 2.0))
    phases = [(g_main, state("G", "r")), (yellow, state("y", "r")),
              (all_red, state("r", "r")),
              (g_cross, state("r", "G")), (yellow, state("r", "y")),
              (all_red, state("r", "r"))]
    lines = [f'    <tlLogic id="{TLS_ID}" type="static" programID="custom" offset="0">']
    lines += [f'        <phase duration="{d:g}" state="{s}"/>'
              for d, s in phases if d > 0]
    lines.append("    </tlLogic>")
    with open(out_path, "w") as f:
        f.write("<additional>\n" + "\n".join(lines) + "\n</additional>\n")


def _blob(cx: float, cy: float, r: float, rng, n: int = 14,
          roughness: float = 0.35) -> str:
    """유기적인(울퉁불퉁한) 수관 실루엣 다각형을 만든다.

    실제 나무를 위에서 내려다보면 완전한 원이 아니라 잎 뭉치들이
    만드는 불규칙한 테두리다. 꼭짓점마다 반지름을 무작위로 흔들되,
    이웃 꼭짓점과 평균을 내서(smoothing) 뾰족하지 않고 부드럽게
    울퉁불퉁한 실루엣을 얻는다.
    """
    import math
    raw = [1.0 + rng.uniform(-roughness, roughness) for _ in range(n)]
    sm = [(raw[i - 1] + raw[i] + raw[(i + 1) % n]) / 3.0 for i in range(n)]
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rr = r * sm[i]
        pts.append(f"{cx + rr * math.cos(a):.2f},{cy + rr * math.sin(a):.2f}")
    return " ".join(pts)


def _write_scenery(out_dir: str, road: dict, cross_x: float = None):
    """도로변 나무를 highway.scenery.xml (additional 파일)로 생성.

    항공뷰(top-down) 조경처럼 보이도록 나무 한 그루를 여러 겹으로 그린다:
      layer 1  그림자  : 수관을 남동쪽으로 살짝 밀어낸 반투명 검정 blob
                         (해가 북서쪽에 있는 항공사진 느낌)
      layer 2  수관 몸통: 중간톤 초록의 불규칙 실루엣
      layer 3  잎 뭉치  : 수관 안에 어두운/짙은 클럼프 1~2개
                         → 캐노피의 입체적인 질감
      layer 4  하이라이트: 북서쪽으로 치우친 밝은 초록 작은 blob
    배치도 실제 조경처럼: 단독목 / 2~3그루 군락 / 낮은 관목이 섞인다.
    SCENERY["trees"]가 False면 빈 additional 파일 생성.

    cross_x가 주어지면 그 x 좌표 부근(교차로 구역)에는 나무를 심지 않는다.
    남북 도로가 지나가는 자리라 나무가 도로 위에 겹쳐 보이기 때문이다.
    """
    import random
    path = os.path.join(out_dir, "highway.scenery.xml")
    cfg = SCENERY if isinstance(SCENERY, dict) else {}
    if not cfg.get("trees", False):
        with open(path, "w") as f:
            f.write("<additional/>\n")
        return

    rng = random.Random(int(cfg.get("seed", 7)))
    spacing = float(cfg.get("tree_spacing", 45.0))
    jitter = float(cfg.get("tree_jitter", 0.5))
    side = float(cfg.get("side_offset", 6.0))
    length = float(road["length"])
    road_bottom = -3.2 * int(road["num_lanes"])
    # 교차로 주변 이 거리(m) 안에는 나무를 심지 않는다 (남북 도로 통과 구간)
    clear_radius = float(cfg.get("intersection_clear", 40.0))

    # (몸통, 어두운 클럼프, 밝은 하이라이트) 초록 팔레트 — 수종별 톤
    palettes = [
        ("46,110,44", "32,84,32", "96,158,84"),
        ("38,98,40",  "26,72,28", "84,146,74"),
        ("56,122,50", "40,92,38", "110,168,92"),
        ("30,88,36",  "20,64,26", "76,138,70"),
    ]
    bush_greens = ["70,124,58", "84,136,64", "62,112,52"]

    lines = ["<additional>"]
    tid = [0]

    def in_intersection(x):
        """교차로 통과 구역이면 True — 이 자리에는 나무를 심지 않는다."""
        return cross_x is not None and abs(x - cross_x) < clear_radius

    def draw_tree(x, y, r):
        if in_intersection(x):
            return
        body, dark, light = rng.choice(palettes)
        i = tid[0]; tid[0] += 1
        # 그림자 (남동쪽 +x, -y 로 오프셋, 반투명)
        lines.append(f'    <poly id="sh_{i}" color="20,26,18,70" fill="1" '
                     f'layer="1" shape="{_blob(x + r*0.35, y - r*0.35, r*1.02, rng)}"/>')
        # 수관 몸통
        lines.append(f'    <poly id="tr_{i}" color="{body}" fill="1" '
                     f'layer="2" shape="{_blob(x, y, r, rng)}"/>')
        # 잎 뭉치 1~2개 (어두운 톤, 수관 내부 무작위 위치)
        for k in range(rng.randint(1, 2)):
            ox = rng.uniform(-0.35, 0.35) * r
            oy = rng.uniform(-0.35, 0.35) * r
            lines.append(f'    <poly id="cl_{i}_{k}" color="{dark}" fill="1" '
                         f'layer="3" shape="{_blob(x + ox, y + oy, r * rng.uniform(0.4, 0.6), rng, n=10)}"/>')
        # 하이라이트 (북서쪽 -x, +y — 그림자 반대 방향)
        lines.append(f'    <poly id="hl_{i}" color="{light}" fill="1" '
                     f'layer="4" shape="{_blob(x - r*0.28, y + r*0.28, r*0.34, rng, n=8, roughness=0.25)}"/>')

    def draw_bush(x, y):
        if in_intersection(x):
            return
        i = tid[0]; tid[0] += 1
        g = rng.choice(bush_greens)
        r = rng.uniform(1.0, 1.8)
        lines.append(f'    <poly id="bsh_{i}" color="20,26,18,55" fill="1" '
                     f'layer="1" shape="{_blob(x + 0.3, y - 0.3, r, rng, n=8)}"/>')
        lines.append(f'    <poly id="bs_{i}" color="{g}" fill="1" '
                     f'layer="2" shape="{_blob(x, y, r, rng, n=8)}"/>')

    for side_sign, base_y in [(+1, side), (-1, road_bottom - side)]:
        x = rng.uniform(0, spacing)
        while x < length:
            y = base_y + side_sign * rng.uniform(0, 4.0)
            roll = rng.random()
            if roll < 0.12:
                draw_bush(x, y)                        # 15%: 낮은 관목
            elif roll < 0.67:
                draw_tree(x, y, rng.uniform(2.6, 4.6))  # 55%: 단독목 (실제 수관 스케일)
            else:
                # 25%: 2~3그루 군락 (겹치듯 붙여 심어 숲 덩어리 느낌)
                n = rng.randint(2, 4)
                for _ in range(n):
                    gx = x + rng.uniform(-5.5, 5.5)
                    gy = y + side_sign * rng.uniform(-2.0, 4.5)
                    draw_tree(gx, gy, rng.uniform(2.2, 3.8))
                x += spacing * 0.4                     # 군락 뒤엔 간격 조금 더
            x += spacing * rng.uniform(1.0 - jitter, 1.0 + jitter)
    lines.append("</additional>")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")