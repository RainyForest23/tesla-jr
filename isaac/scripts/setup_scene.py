"""
Tesla Jr. - 씬 조명 + 장애물 세팅
=================================================================
stereo 카메라가 실제로 볼 것이 있도록 조명을 강화하고 로봇 전방에
색깔 큐브(장애물/타겟)를 배치한다. 이 큐브들은 추후 depth/YOLO/Nav2
단계의 인식 대상으로도 사용한다.

전제: 로봇이 원점에서 +X 방향을 보고 있음 (import_t870.py 기본).

레이아웃 선택 (맨 아래 LAYOUT 변수):
  "simple"  : 완만 S자 회피용 큐브 2개 (5,+0.7)/(9,-0.7)
  "reverse" : 전방 벽 + 뒤쪽 목표. 정면이 막혀 Ackermann이 후진(K턴)으로만
              풀 수 있음. 벽이 전방 FOV라 카메라가 즉시 맵핑 -> 안정적.
              목표 (-2.0,0.5) 로 보내면 후진 발생.
  "hairpin" : 헤어핀(U턴) 통로. 개념상 3점턴 유발하나 전방 카메라-only 로는
              시작 시 옆/뒤 격벽을 못 봐서 글로벌 플랜이 최단직선이 됨(미로 부적합).
              쓰려면 먼저 통로를 텔레옵 주행해 costmap 을 채운 뒤 목표 전송 필요.

사용법 (Isaac Sim Script Editor, Play 중이어도 됨):
  exec(open("/workspace/isaac/scripts/setup_scene.py").read())
"""

import numpy as np
import omni.usd
from pxr import UsdLux, UsdGeom, Gf
from omni.isaac.core.objects import FixedCuboid


def setup_lighting():
    stage = omni.usd.get_context().get_stage()

    # Dome light 강화 (전반 ambient)
    dome_path = "/World/DomeLight"
    dome = UsdLux.DomeLight.Get(stage, dome_path)
    if not dome:
        dome = UsdLux.DomeLight.Define(stage, dome_path)
    dome.CreateIntensityAttr(1500.0)

    # Distant light (태양광) 추가 - 그림자/대비 생성
    sun_path = "/World/SunLight"
    sun = UsdLux.DistantLight.Get(stage, sun_path)
    if not sun:
        sun = UsdLux.DistantLight.Define(stage, sun_path)
        # 비스듬히 내리쬐도록 회전
        xform = UsdGeom.Xformable(sun.GetPrim())
        xform.AddRotateXYZOp().Set(Gf.Vec3f(-60.0, 15.0, 0.0))
    sun.CreateIntensityAttr(3000.0)
    print("조명 설정 완료 (Dome 1500 + Sun 3000)")


def _clear_obstacles(stage):
    """이름이 obstacle_ 로 시작하는 모든 prim 제거 (레이아웃 재구성용)."""
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return
    to_remove = [c.GetPath() for c in world.GetChildren()
                 if c.GetName().startswith("obstacle_")]
    for p in to_remove:
        stage.RemovePrim(p)


def _spawn_cubes(coords, color, counter):
    """(x,y) 리스트를 FixedCuboid 로 스폰. counter[0] 로 고유 인덱스 부여."""
    for (x, y) in coords:
        i = counter[0]
        FixedCuboid(
            prim_path=f"/World/obstacle_{i}",
            name=f"obstacle_{i}",
            position=np.array([x, y, 0.5]),
            scale=np.array([0.5, 0.5, 1.0]),
            color=np.array(color),
        )
        counter[0] += 1


def _wall(x0, y0, x1, y1, spacing=0.6):
    """(x0,y0)->(x1,y1) 직선을 따라 큐브 중심 좌표 리스트 생성."""
    length = float(np.hypot(x1 - x0, y1 - y0))
    n = max(1, int(round(length / spacing)))
    return [(x0 + (x1 - x0) * t / n, y0 + (y1 - y0) * t / n)
            for t in range(n + 1)]


def add_obstacles_simple():
    stage = omni.usd.get_context().get_stage()
    _clear_obstacles(stage)
    counter = [0]
    # Ackermann 최소반경(~0.95m)으로 피할 수 있게 치우치고 넓게: 완만한 S 회피.
    _spawn_cubes([(5.0, 0.7)], (0.9, 0.1, 0.1), counter)   # 빨강 (좌 -> 우 회피)
    _spawn_cubes([(9.0, -0.7)], (0.1, 0.3, 0.9), counter)  # 파랑 (우 -> 좌 회피)
    print(f"[simple] 장애물 {counter[0]}개: (5,+0.7)/(9,-0.7)")


def add_obstacles_snake():
    """ㄹ자(스네이크) 미로: 수평 3차선을 좌/우 교대로 연결. 동→북→서→북→동
    (4번 꺾음). 차선폭 3.0m. persistent costmap + MPPI 다중코너 주행 검증.
    목표 (5.0, 6.0).

        lane3 (y4.5~7.5) ──────────→ [goal 5,6]   (동)
              ↑ gap x[0,2]
        lane2 (y1.5~4.5) ←────────── 서
                          gap x[4,6] ↑
        [robot]→ lane1 (y-1.5~1.5) ─→ 동
    """
    stage = omni.usd.get_context().get_stage()
    _clear_obstacles(stage)
    counter = [0]
    gray = (0.6, 0.6, 0.6)
    walls = []
    walls += _wall(0.0, -1.5, 6.0, -1.5)   # 바닥 경계
    walls += _wall(0.0,  1.5, 4.0,  1.5)   # div1 (우측 x[4,6] 열림 -> 위로)
    walls += _wall(2.0,  4.5, 6.0,  4.5)   # div2 (좌측 x[0,2] 열림 -> 위로)
    walls += _wall(0.0,  7.5, 6.0,  7.5)   # 천장 경계
    walls += _wall(0.0,  1.5, 0.0,  7.5)   # 좌측 경계 (lane1 입구는 열림)
    walls += _wall(6.0, -1.5, 6.0,  7.5)   # 우측 경계
    _spawn_cubes(walls, gray, counter)
    print(f"[snake] ㄹ자 미로 벽 큐브 {counter[0]}개 (차선폭 3.0m). 목표 (5,6).")
    print("  send_goal.sh 5 6   -> 동→북→서→북→동 4코너 주행")


def add_obstacles_corner():
    """L자 코너 + 후진 유발. 로봇이 진입차선을 직진하다 정면 벽(x=3.6)에 막혀
    좌측(+Y)으로 90° 꺾어 출구차선으로 진입해야 한다. 코너 통로가 좁아 최소
    회전반경(~1.0m)으로는 한 번에 못 돌고 전진->후진->전진(3점턴)이 필요.
    목표 (2.6, 4.0). 정면 벽 = '반드시 피해야 하는 앞 장애물'.

        inner(x=1.6)│   │outer/정면벽(x=3.6)
                    │   │
        ─top(y=1.2)─┘   │   <- 진입차선 윗벽(짧게)
        [robot]→        │      진입(+X) -> 코너 -> 출구(+Y)
        ────bottom(y=-1.2)──┘
    """
    stage = omni.usd.get_context().get_stage()
    _clear_obstacles(stage)
    counter = [0]
    gray = (0.6, 0.6, 0.6)
    red = (0.9, 0.1, 0.1)
    # 차선폭 2.4m: MPPI로 후진(3점턴) 유발. (RPP는 여기서 wedge했음)
    bottom = _wall(0.0, -1.2, 3.8, -1.2)    # 진입차선 바닥벽
    outer = _wall(3.8, -1.2, 3.8, 5.0)      # 정면벽 + 출구차선 바깥벽
    topentry = _wall(0.0, 1.2, 1.4, 1.2)    # 진입차선 윗벽 (짧게)
    inner = _wall(1.4, 1.2, 1.4, 5.0)       # 출구차선 안쪽벽 (폭 2.4m)
    _spawn_cubes(bottom, gray, counter)
    _spawn_cubes(outer, red, counter)       # 정면벽 빨강 강조
    _spawn_cubes(topentry, gray, counter)
    _spawn_cubes(inner, gray, counter)
    print(f"[corner] 벽 큐브 {counter[0]}개. 정면벽(x=3.8) 만나 좌 90° 꺾어 진입(폭 2.4m).")
    print("  목표 (2.6,4.0). send_goal.sh 2.6 4.0  -> MPPI 후진(3점턴) 기대")


def add_obstacles_gate():
    """전방-대각선 게이트. 기둥을 ±40~50° 방향에 둬서 front 단독 화각(±30°)
    밖이지만 좌/우 카메라(중심 ±60°) FOV 안에 들도록 배치 -> 3카메라 통합
    costmap 효과 검증용. 로봇은 게이트 사이를 통과해 목표 (6,0) 으로 주행.

        (2.0,+1.3) 빨강                 [목표 6,0]
        [robot]→            (4.0,+0.9) 빨강(살짝 우회)
        (2.0,-1.3) 파랑
    """
    stage = omni.usd.get_context().get_stage()
    _clear_obstacles(stage)
    counter = [0]
    # 게이트 기둥 (atan(1.3/2.0)≈33°, 좌/우 카메라 FOV 안)
    _spawn_cubes([(2.0, 1.3)], (0.9, 0.1, 0.1), counter)   # 좌 기둥
    _spawn_cubes([(2.0, -1.3)], (0.1, 0.3, 0.9), counter)  # 우 기둥
    # 통과 후 살짝 우회시킬 장애물
    _spawn_cubes([(4.0, 0.9)], (0.9, 0.1, 0.1), counter)
    print(f"[gate] 장애물 {counter[0]}개: 게이트(2,±1.3)+(4,+0.9). 목표 (6,0).")
    print("  goal 예: ros2 topic pub --once /goal_pose geometry_msgs/msg/"
          "PoseStamped \"{header:{frame_id:'odom'},pose:{position:{x:6.0,y:0.0}}}\"")


def add_obstacles_reverse():
    """전방 벽 + 뒤쪽 목표 = 후진(K턴) 유발.

    로봇 원점(+X) 앞 x=1.2 에 폭 넓은 벽(y:-3~+3)을 세워 전진/전진선회를
    완전히 막는다. 목표를 뒤쪽 (-2.0,0.5) 로 주면 Ackermann은 제자리 회전이
    안 되므로 Smac REEDS_SHEPP 가 '후진 -> (필요시 전진)' 으로 경로를 푼다.
    벽이 전방 카메라 FOV 안이라 costmap 에 즉시 반영됨(미관측 문제 없음).

        [wall x=1.2, y:-3~+3]
                │
        [robot]→│      목표 (-2.0,0.5) 는 로봇 뒤쪽
    """
    stage = omni.usd.get_context().get_stage()
    _clear_obstacles(stage)
    counter = [0]
    wall = _wall(1.2, -3.0, 1.2, 3.0)          # 전방 가로벽 (충분히 넓게)
    _spawn_cubes(wall, (0.9, 0.1, 0.1), counter)  # 빨강
    print(f"[reverse] 전방 벽 큐브 {counter[0]}개 (x=1.2). "
          f"목표 (-2.0,0.5) 로 보내면 후진(K턴) 발생.")
    print("  goal 예: ros2 topic pub --once /goal_pose geometry_msgs/msg/"
          "PoseStamped \"{header:{frame_id:'odom'},pose:{position:{x:-2.0,y:0.5}}}\"")


def add_obstacles_hairpin():
    """헤어핀(U턴) 통로. 로봇 원점(+X) -> 상단 차선 직진 -> 끝에서 U턴 ->
    하단 차선으로 복귀. 차선폭 2.0m(통과 가능), U턴 공간은 좁아 회전직경 2.0m
    로는 한번에 못 돌아 3점턴(후진) 발생. 목표 (1.0,-2.0) 로 보낼 것.

      top wall  y=+1.0  ───────────────┐  (far cap)
      [robot]→  상단차선(y~0)          │
      divider   y=-1.0 ──────┘(tip x=5)│
                하단차선(y~-2)          │
      bot wall  y=-3.0  ───────────────┘
    """
    stage = omni.usd.get_context().get_stage()
    _clear_obstacles(stage)
    counter = [0]

    X_END = 6.5      # 통로 길이 (far cap 위치)
    TIP = 5.0        # divider 끝(여기를 돌아 U턴)
    gray = (0.6, 0.6, 0.6)
    red = (0.9, 0.1, 0.1)

    # 상단 벽 (y=+1.0), 하단 벽 (y=-3.0): 통로 바깥 경계
    top = _wall(0.0, 1.0, X_END, 1.0)
    bot = _wall(0.0, -3.0, X_END, -3.0)
    # 가운데 격벽 (y=-1.0, x: 0 -> TIP): 두 차선 분리, tip에서 끊겨 U턴 통로 확보
    divider = _wall(0.0, -1.0, TIP, -1.0)
    # far cap (x=X_END, y: -3 -> +1): 끝을 막아 U턴 강제
    cap = _wall(X_END, -3.0, X_END, 1.0)

    _spawn_cubes(top, gray, counter)
    _spawn_cubes(bot, gray, counter)
    _spawn_cubes(divider, gray, counter)
    _spawn_cubes(cap, red, counter)   # 막다른 끝은 빨강으로 강조
    print(f"[hairpin] 벽 큐브 {counter[0]}개. 목표를 (1.0,-2.0)로 보내면 "
          f"끝(x={X_END})에서 U턴하며 후진(3점턴) 발생.")
    print("  goal 예: ros2 topic pub --once /goal_pose geometry_msgs/msg/"
          "PoseStamped \"{header:{frame_id:'odom'},pose:{position:{x:1.0,y:-2.0}}}\"")


# ─── 레이아웃 선택 ────────────────────────────────────────────────
LAYOUT = "snake"   # "simple" | "gate" | "corner" | "snake" | "reverse" | "hairpin"

setup_lighting()
if LAYOUT == "simple":
    add_obstacles_simple()
elif LAYOUT == "gate":
    add_obstacles_gate()
elif LAYOUT == "corner":
    add_obstacles_corner()
elif LAYOUT == "snake":
    add_obstacles_snake()
elif LAYOUT == "reverse":
    add_obstacles_reverse()
elif LAYOUT == "hairpin":
    add_obstacles_hairpin()
else:
    print(f"!!! 알 수 없는 LAYOUT={LAYOUT}")
print(f">>> 씬 세팅 완료 (LAYOUT={LAYOUT}). 큐브 안 보이면 Stop -> Play 한번.")
