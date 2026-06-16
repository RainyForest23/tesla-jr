"""
Tesla Jr. - 로봇 스폰 스크립트
=================================================================
빈 씬에 다음을 세팅한다:
  - 물리 씬 (World / SimulationContext)
  - 기본 Ground Plane
  - Dome Light (카메라용 조명)
  - Nova Carter (stereo camera 탑재 모델) at /World/Carter

stereo camera 기반 프로젝트이므로 stereo 가 달린 nova_carter_sensors 를
우선 사용한다. 에셋 경로는 후보 목록에서 자동 탐지.

사용법 (Isaac Sim Script Editor):
  exec(open("/workspace/isaac/scripts/spawn_robot.py").read())
  # -> 그 다음 Play, 이어서 sim_ros2_bridge.py 실행

주의: 인터넷/Nucleus 에서 에셋을 받아오므로 첫 로드는 수십 초 걸릴 수 있음.
"""

import omni.client
import omni.usd
from pxr import UsdLux, UsdPhysics, Gf

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim

ROBOT_PRIM_PATH = "/World/Carter"
SPAWN_HEIGHT = 0.3  # 지면 위로 살짝 띄워 스폰 (바닥 통과 방지)

# stereo 가 달린 모델 우선. 위에서부터 존재하는 첫 경로 사용.
ROBOT_CANDIDATES = [
    "/Isaac/Robots/Carter/nova_carter_sensors.usd",
    "/Isaac/Robots/Carter/nova_carter.usd",
    "/Isaac/Robots/Carter/carter_v2.usd",
    "/Isaac/Robots/Carter/carter_v1.usd",
]


def _exists(url: str) -> bool:
    try:
        result, _ = omni.client.stat(url)
        return result == omni.client.Result.OK
    except Exception:
        return False


def main():
    assets_root = get_assets_root_path()
    if assets_root is None:
        print("!!! 에셋 루트를 찾을 수 없음 (Nucleus/인터넷 연결 확인)")
        return
    print(f"에셋 루트: {assets_root}")

    # 1) World + 물리 + Ground Plane
    world = World.instance()
    if world is None:
        world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    # PhysicsScene 명시적 생성 (GUI 모드에선 자동 생성 안 됨)
    if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)
        print("PhysicsScene 추가됨")

    world.scene.add_default_ground_plane()
    print("Ground Plane 추가됨")

    # 2) Dome Light (카메라 시야 확보)
    light_path = "/World/DomeLight"
    if not stage.GetPrimAtPath(light_path).IsValid():
        dome = UsdLux.DomeLight.Define(stage, light_path)
        dome.CreateIntensityAttr(1000.0)
        print("Dome Light 추가됨")

    # 3) 로봇 에셋 경로 자동 탐지
    robot_usd = None
    for cand in ROBOT_CANDIDATES:
        full = assets_root + cand
        if _exists(full):
            robot_usd = full
            print(f"로봇 에셋 발견: {cand}")
            break
    if robot_usd is None:
        print("!!! 로봇 에셋을 못 찾음. 후보 경로:")
        for c in ROBOT_CANDIDATES:
            print("   ", c)
        return

    # 4) 로봇 스폰
    if stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid():
        print(f"이미 존재함: {ROBOT_PRIM_PATH} (재사용)")
    else:
        add_reference_to_stage(usd_path=robot_usd, prim_path=ROBOT_PRIM_PATH)
        print(f"로봇 스폰됨: {ROBOT_PRIM_PATH}")

    XFormPrim(ROBOT_PRIM_PATH).set_world_pose(
        position=[0.0, 0.0, SPAWN_HEIGHT]
    )

    # 5) 물리 초기화 (GUI 에선 실패해도 Play 가 초기화하므로 무시)
    try:
        world.reset()
        print("물리 초기화 완료")
    except Exception as e:
        print(f"world.reset() 건너뜀 (Play 가 초기화함): {e}")

    print(">>> 스폰 완료. 이제 Play 를 누르고 sim_ros2_bridge.py 를 실행하세요.")
    print(f">>> 로봇 prim 경로 = {ROBOT_PRIM_PATH}")
    print(">>> Stage 패널에서 카메라 prim 경로도 확인해두면 다음 단계(stereo publish)에 사용합니다.")


main()
