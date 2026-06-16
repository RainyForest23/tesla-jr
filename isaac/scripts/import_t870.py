"""
Tesla Jr. - T870 URDF 임포트 (Phase 1)
=================================================================
urdf/t870.urdf 를 Isaac Sim 에 임포트하고 ground/조명/물리와 함께 세팅.
Carter(차동구동/캐스터) 대체. Ackermann 조향.

사용법 (Isaac Sim Script Editor, 새 씬 권장):
  exec(open("/workspace/isaac/scripts/import_t870.py").read())
  # -> Play 눌러 바닥에 서는지 확인

임포트 후 조인트:
  rear_left/right_wheel_joint    구동(뒤)
  front_left/right_steer_joint   조향(앞, revolute ±0.6rad)
  front_left/right_wheel_joint   앞바퀴 rolling
다음 단계: Ackermann 컨트롤러 + 센서 재배선.
"""

import omni.usd
import omni.kit.commands
from pxr import UsdLux, UsdPhysics, Gf, Sdf
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.prims import XFormPrim

URDF_PATH = "/workspace/urdf/t870.urdf"
ROBOT_PRIM = "/t870"          # 임포터 기본 (robot name = t870)
SPAWN_HEIGHT = 0.2


def setup_world():
    # File>New 후 이전 세션의 World 싱글톤이 남아 새 씬에 ground 가 안 생기는
    # 문제 방지 -> 싱글톤을 비우고 현재 스테이지로 새로 생성.
    World.clear_instance()
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)
        print("PhysicsScene 추가됨")
    # ground 를 직접 생성 (World.scene 경유보다 견고)
    from omni.isaac.core.objects.ground_plane import GroundPlane
    if not stage.GetPrimAtPath("/World/GroundPlane").IsValid():
        GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)
        print("Ground Plane 추가됨")
    light = "/World/DomeLight"
    if not stage.GetPrimAtPath(light).IsValid():
        UsdLux.DomeLight.Define(stage, light).CreateIntensityAttr(1500.0)
    return world, stage


def import_urdf():
    enable_extension("omni.importer.urdf")
    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False      # 카메라 mount 링크 유지
    cfg.import_inertia_tensor = True    # URDF 관성 사용
    cfg.fix_base = False                # 이동 로봇
    cfg.self_collision = False
    cfg.distance_scale = 1.0
    cfg.make_default_prim = False
    result = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=URDF_PATH, import_config=cfg
    )
    print(f"임포트 결과: {result}")


def main():
    world, stage = setup_world()
    import_urdf()

    # 임포트된 로봇 prim 찾기 (t870)
    robot_path = None
    for cand in [ROBOT_PRIM, "/World/t870"]:
        if stage.GetPrimAtPath(Sdf.Path(cand)).IsValid():
            robot_path = cand
            break
    if robot_path is None:
        # 이름에 t870 포함된 최상위 prim 탐색
        for p in stage.Traverse():
            if "t870" in p.GetName().lower() and str(p.GetPath()).count("/") == 1:
                robot_path = str(p.GetPath())
                break
    if robot_path is None:
        print("!!! 임포트된 t870 prim 을 못 찾음. Stage 패널에서 경로 확인 필요")
        return
    print(f"로봇 prim: {robot_path}")

    XFormPrim(robot_path).set_world_pose(position=[0.0, 0.0, SPAWN_HEIGHT])
    try:
        world.reset()
    except Exception as e:
        print(f"world.reset() 건너뜀: {e}")

    print(">>> 임포트 완료. Play 를 눌러 바닥에 서는지 확인.")
    print(f">>> 로봇 경로 = {robot_path}")


main()
