"""
Tesla Jr. - T870 바퀴 마찰/접지 진단 + 고마찰 재질 적용
=================================================================
뒷바퀴가 헛도는 문제(traction) 진단. base_link 높이 출력으로 차체 접지 여부
확인 + 바퀴/바닥에 고마찰 PhysicsMaterial 적용.

사용법 (Isaac Sim Script Editor, Play 중):
  exec(open("/workspace/isaac/scripts/fix_friction.py").read())
"""

import omni.usd
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.prims import GeometryPrim

stage = omni.usd.get_context().get_stage()

# 1) base_link 높이 (차체 접지 여부)
dc = _dynamic_control.acquire_dynamic_control_interface()
b = dc.get_rigid_body("/World/t870/base_link")
if b:
    p = dc.get_rigid_body_pose(b)
    print(f"base_link z = {p.p.z:.3f}  (바퀴로 서면 ~0.12 근처, 음수면 차체 접지)")

# 2) 재질 2종: 뒷바퀴=구동용 고마찰, 앞바퀴=조향용 저마찰(scrub-lock 방지)
mat_rear = PhysicsMaterial(
    prim_path="/World/Physics/rear_high_friction",
    static_friction=1.5, dynamic_friction=1.2, restitution=0.0,
)
mat_front = PhysicsMaterial(
    prim_path="/World/Physics/front_low_friction",
    static_friction=0.3, dynamic_friction=0.25, restitution=0.0,
)

# 3) 바퀴 collider 에 적용 (뒤=고마찰, 앞=저마찰)
wheel_mat = {
    "rear_left_wheel": mat_rear, "rear_right_wheel": mat_rear,
    "front_left_wheel": mat_front, "front_right_wheel": mat_front,
}
mat = mat_rear  # 바닥엔 고마찰
for w, m in wheel_mat.items():
    applied = False
    for path in [f"/World/t870/{w}/collisions", f"/World/t870/{w}"]:
        if not stage.GetPrimAtPath(path).IsValid():
            continue
        try:
            GeometryPrim(path).apply_physics_material(m)
            tag = "저마찰" if m is mat_front else "고마찰"
            print(f"마찰 적용({tag}): {path}")
            applied = True
            break
        except Exception as e:
            print(f"  {path} 실패: {e}")
    if not applied:
        print(f"!!! {w} 적용 실패")

# 4) 바닥에도 적용
for gpath in ["/World/GroundPlane", "/World/GroundPlane/CollisionPlane",
              "/World/defaultGroundPlane"]:
    if stage.GetPrimAtPath(gpath).IsValid():
        try:
            GeometryPrim(gpath).apply_physics_material(mat)
            print(f"마찰 적용(바닥): {gpath}")
        except Exception as e:
            print(f"  바닥 {gpath} 실패: {e}")

print(">>> 완료. cmd_vel 로 다시 전진 테스트.")
