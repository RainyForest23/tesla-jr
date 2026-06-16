"""
Tesla Jr. - 원클릭 씬 구성 런처
=================================================================
빈 씬에서 아래를 순서대로 구성한다 (Play 누르기 전 단계까지):
  1. spawn_robot.py            로봇 + ground + light + physics
  2. setup_scene.py            조명 강화 + 장애물 큐브
  3. stereo_camera_publish.py  stereo 카메라 publish 그래프
  4. teleop_cmdvel.py          /cmd_vel 주행 그래프

사용법 (Isaac Sim Script Editor, 새 씬에서):
  exec(open("/workspace/isaac/scripts/launch_all.py").read())
  # -> Play(▶) 를 누른다
  # -> 그 다음 상태 publish 시작:
  exec(open("/workspace/isaac/scripts/sim_ros2_bridge.py").read())

주의: sim_ros2_bridge.py 는 Play 이후에 실행해야 한다
      (dynamic_control 핸들이 물리 시작 후에 유효).
"""

_BASE = "/workspace/isaac/scripts/"
_STEPS = [
    "spawn_robot.py",
    "setup_scene.py",
    "stereo_camera_publish.py",
    "teleop_cmdvel.py",
]

for _s in _STEPS:
    print(f"\n========== {_s} ==========")
    try:
        exec(open(_BASE + _s).read())
    except Exception as _e:
        print(f"!!! {_s} 실행 중 오류: {_e}")
        raise

print("\n" + "=" * 50)
print(">>> 씬 구성 완료. 이제 Play(▶) 를 누르세요.")
print(">>> Play 후 상태 publish 시작:")
print('>>>   exec(open("/workspace/isaac/scripts/sim_ros2_bridge.py").read())')
print("=" * 50)
