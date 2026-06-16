"""
Tesla Jr. - T870 Ackermann 컨트롤러 (Python)
=================================================================
/cmd_vel (geometry_msgs/Twist) 를 받아 Ackermann 조향/구동으로 변환:
  조향각 steer = atan(wheelBase * w / v)   (앞 조향 조인트 위치 타깃)
  바퀴속도 omega = v / wheelRadius          (뒤 구동 조인트 속도 타깃)
dynamic_control 로 조인트 드라이브 게인 설정 + 타깃 적용.

조인트(DOF):
  0 front_left_steer  1 front_right_steer  (조향, 위치 드라이브)
  2 rear_left_wheel   3 rear_right_wheel   (구동, 속도 드라이브)
  4 front_left_wheel  5 front_right_wheel  (앞 rolling, 자유)

사용법 (Isaac Sim Script Editor, Play 중):
  exec(open("/workspace/isaac/scripts/ackermann_control.py").read())
주행: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear:{x:0.5}, angular:{z:0.3}}"
중지: ack.stop()
"""

import math
import omni.physx
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from omni.isaac.dynamic_control import _dynamic_control

ART_PATH = "/World/t870/base_link"
WHEEL_BASE = 0.65
WHEEL_RADIUS = 0.12
MAX_STEER = 0.6          # rad
MIN_SPEED_FOR_STEER = 0.05

STEER_JOINTS = ["front_left_steer_joint", "front_right_steer_joint"]
DRIVE_JOINTS = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
ROLL_JOINTS = ["front_left_wheel_joint", "front_right_wheel_joint"]


class AckermannControl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("t870_ackermann")
        self.v = 0.0
        self.w = 0.0
        self.node.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        # 전용 executor (전역 default executor/다른 spinner 와 충돌 방지)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)

        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.art = None
        self.dofs = {}
        self._configured = False

        self._phys_sub = (
            omni.physx.get_physx_interface()
            .subscribe_physics_step_events(self._on_step)
        )
        print(">>> Ackermann 컨트롤러 시작. /cmd_vel 로 주행.")
        print(">>> 중지: ack.stop()")

    def _cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _acquire(self):
        self.art = self.dc.get_articulation(ART_PATH)
        if self.art == 0:
            return False
        for name in STEER_JOINTS + DRIVE_JOINTS + ROLL_JOINTS:
            self.dofs[name] = self.dc.find_articulation_dof(self.art, name)
        return True

    def _configure_drives(self):
        # 드라이브 게인 설정: 조향=위치(stiffness 큼), 구동=속도(stiffness 0, damping 큼),
        # 앞 rolling=자유(거의 무저항)
        props = self.dc.get_articulation_dof_properties(self.art)
        names = [self.dc.get_dof_name(self.dc.get_articulation_dof(self.art, i))
                 for i in range(self.dc.get_articulation_dof_count(self.art))]
        for i, nm in enumerate(names):
            # 위치제어=stiffness>0, 속도제어=stiffness 0 + damping.
            # 조향 링크 관성이 작아 FORCE 모드 고stiffness는 불안정 -> ACCELERATION 모드
            # (게인이 관성과 무관). FORCE=0, ACCELERATION=1.
            if nm in STEER_JOINTS:
                props["driveMode"][i] = getattr(_dynamic_control, "DRIVE_ACCELERATION", 1)
                props["stiffness"][i] = 500.0
                props["damping"][i] = 50.0
                props["maxEffort"][i] = 1.0e6
            elif nm in DRIVE_JOINTS:
                props["driveMode"][i] = _dynamic_control.DRIVE_FORCE
                props["stiffness"][i] = 0.0
                props["damping"][i] = 1.0e4
                props["maxEffort"][i] = 1.0e5
            elif nm in ROLL_JOINTS:
                # 앞바퀴는 완전 자유 회전이어야 함 (damping>0 이면 브레이크처럼
                # 굴러가는 걸 막아 전진을 방해). stiffness/damping 모두 0.
                props["driveMode"][i] = _dynamic_control.DRIVE_FORCE
                props["stiffness"][i] = 0.0
                props["damping"][i] = 0.0
                props["maxEffort"][i] = 0.0
        self.dc.set_articulation_dof_properties(self.art, props)
        print("드라이브 게인 설정 완료")

    def _on_step(self, dt):
        # /cmd_vel 콜백 처리 (전용 executor, 단일 스레드)
        try:
            self._exec.spin_once(timeout_sec=0.0)
        except Exception:
            pass
        if self.art is None or self.art == 0:
            if not self._acquire():
                return
        if not self._configured:
            try:
                self._configure_drives()
                self._configured = True
            except Exception as e:
                print(f"드라이브 설정 예외: {e}")
                return

        self.dc.wake_up_articulation(self.art)
        v, w = self.v, self.w
        # Ackermann 조향각 (정지 시 0)
        if abs(v) > MIN_SPEED_FOR_STEER:
            steer = math.atan(WHEEL_BASE * w / v)
        else:
            steer = 0.0
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        # 바퀴 축이 +Y -> omega>0 이 +X 전진 (물리적으로 맞음)
        omega = v / WHEEL_RADIUS

        for nm in STEER_JOINTS:
            self.dc.set_dof_position_target(self.dofs[nm], steer)
        for nm in DRIVE_JOINTS:
            self.dc.set_dof_velocity_target(self.dofs[nm], omega)

    def stop(self):
        if self._phys_sub is not None:
            try:
                self._phys_sub.unsubscribe()
            except Exception:
                pass
        self._phys_sub = None
        # 정지
        try:
            if self.art and self.art != 0:
                for nm in DRIVE_JOINTS:
                    self.dc.set_dof_velocity_target(self.dofs[nm], 0.0)
        except Exception:
            pass
        try:
            self._exec.remove_node(self.node)
            self._exec.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        print(">>> Ackermann 컨트롤러 중지")


# ---- 부트스트랩 (이전 인스턴스 정리) ----
import builtins
_reg = getattr(builtins, "_TESLA_ACK", [])
for _o in _reg:
    try:
        _o.stop()
    except Exception:
        pass
_reg.clear()
ack = AckermannControl()
_reg.append(ack)
builtins._TESLA_ACK = _reg
