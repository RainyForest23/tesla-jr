"""
Tesla Jr. - 씬 조명 + 장애물 세팅
=================================================================
stereo 카메라가 실제로 볼 것이 있도록 조명을 강화하고 로봇 전방에
색깔 큐브(장애물/타겟)를 배치한다. 이 큐브들은 추후 depth/YOLO/Nav2
단계의 인식 대상으로도 사용한다.

전제: 로봇이 /World/Carter 에 +X 방향을 보고 있음 (spawn_robot.py 기본).

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


def add_obstacles():
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    # 기존 장애물 제거 (재배치)
    for i in range(8):
        p = f"/World/obstacle_{i}"
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)

    # Ackermann 최소반경(~0.95m) 으로 피할 수 있게 한쪽으로 치우치고 넓게 벌림.
    # 로봇은 직선 경로에서 완만한 S 로 회피.
    # (x, y, RGB)
    props = [
        (5.0,  0.7, (0.9, 0.1, 0.1)),   # 빨강 (살짝 좌 -> 우로 완만 회피)
        (9.0, -0.7, (0.1, 0.3, 0.9)),   # 파랑 (살짝 우 -> 좌로 완만 회피)
    ]
    for i, (x, y, color) in enumerate(props):
        FixedCuboid(
            prim_path=f"/World/obstacle_{i}",
            name=f"obstacle_{i}",
            position=np.array([x, y, 0.5]),
            scale=np.array([0.5, 0.5, 1.0]),
            color=np.array(color),
        )
        print(f"  장애물 {i}: pos=({x},{y})")
    print("장애물 배치 완료 (완만 회피용: 5,+0.7 / 9,-0.7)")


setup_lighting()
add_obstacles()
print(">>> 씬 세팅 완료. 카메라 화면이 밝아지고 큐브들이 보일 것.")
print(">>> 안 보이면 Stop -> Play 한번.")
