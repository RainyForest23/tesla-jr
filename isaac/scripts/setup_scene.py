"""
Tesla Jr. - 씬 조명 + 장애물 세팅
=================================================================
stereo 카메라가 실제로 볼 것이 있도록 조명을 강화하고 로봇 전방에
색깔 큐브(장애물/타겟)를 배치한다. 이 큐브들은 추후 depth/YOLO/Nav2
단계의 인식 대상으로도 사용한다.

전제: 로봇이 원점에서 +X 방향을 보고 있음 (import_t870.py 기본).

레이아웃 선택 (맨 아래 LAYOUT 변수):
  "simple"  : 완만 S자 회피용 큐브 2개 (5,+0.7)/(9,-0.7)
  "hairpin" : 헤어핀(U턴) 통로 - 통로폭<회전직경 이라 3점턴(후진) 유발
              목표 (1.0,-2.0) 로 보내면 끝에서 U턴하며 후진 발생.

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
LAYOUT = "hairpin"   # "simple" | "hairpin"

setup_lighting()
if LAYOUT == "simple":
    add_obstacles_simple()
elif LAYOUT == "hairpin":
    add_obstacles_hairpin()
else:
    print(f"!!! 알 수 없는 LAYOUT={LAYOUT}")
print(f">>> 씬 세팅 완료 (LAYOUT={LAYOUT}). 큐브 안 보이면 Stop -> Play 한번.")
