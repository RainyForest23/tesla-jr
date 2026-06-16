"""
Tesla Jr. - T870 원클릭 씬 구성 런처 (Phase 1)
=================================================================
빈 씬에서 T870 자율주행 시뮬 환경을 구성:
  1. import_t870.py    ground+조명+물리 + T870 임포트 (바닥 기립)
  2. t870_camera.py    전방 카메라 + depth/포인트클라우드 그래프

사용법 (Isaac Sim Script Editor, 새 씬 File>New 에서):
  exec(open("/workspace/isaac/scripts/launch_t870.py").read())
  # -> Play(▶)
  # -> 주행 제어 + odom 발행:
  exec(open("/workspace/isaac/scripts/ackermann_control.py").read())
  exec(open("/workspace/isaac/scripts/sim_ros2_bridge.py").read())

주의: ackermann_control / sim_ros2_bridge 는 Play 이후 실행.
      장애물이 필요하면 setup_scene.py 도 실행(조명+큐브).
"""

_BASE = "/workspace/isaac/scripts/"
for _s in ["import_t870.py", "t870_camera.py"]:
    print(f"\n========== {_s} ==========")
    try:
        exec(open(_BASE + _s).read())
    except Exception as _e:
        print(f"!!! {_s} 오류: {_e}")
        raise

print("\n" + "=" * 50)
print(">>> T870 구성 완료. Play(▶) 누르고:")
print('>>>   exec(open("/workspace/isaac/scripts/ackermann_control.py").read())')
print('>>>   exec(open("/workspace/isaac/scripts/sim_ros2_bridge.py").read())')
print(">>> 장애물: setup_scene.py (단, T870 전방 비우려면 큐브 위치 조정)")
print("=" * 50)
