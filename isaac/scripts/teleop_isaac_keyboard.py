"""
Tesla Jr. - Isaac Sim 창 안에서 키보드 운전
=================================================================
Isaac Sim 뷰포트에 포커스를 둔 채 키보드로 Carter 를 운전한다.
carb 입력으로 키 상태를 받아 /cmd_vel (Twist) 을 발행 → 기존 teleop
그래프(teleop_cmdvel.py)가 바퀴를 굴린다.

  W / ↑ : 전진      S / ↓ : 후진
  A / ← : 좌회전    D / → : 우회전
  (여러 키 동시 입력 가능. 떼면 정지)

사용법 (Isaac Sim Script Editor, Play 중):
  exec(open("/workspace/isaac/scripts/teleop_isaac_keyboard.py").read())
  # 그 다음 Isaac Sim "뷰포트"를 클릭해 포커스를 주고 W/A/S/D 또는 방향키

중지:
  key_teleop.stop()

참고: 뷰포트 카메라 조작(WASD)과 겹칠 수 있으면 방향키 사용.
"""

import carb.input
import omni.appwindow
import omni.physx
import rclpy
from geometry_msgs.msg import Twist

LIN_SPEED = 1.5   # m/s  (느리면 올리기)
ANG_SPEED = 2.0   # rad/s


class IsaacKeyTeleop:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("isaac_key_teleop")
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

        self.pressed = set()
        app_window = omni.appwindow.get_default_app_window()
        self._keyboard = app_window.get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._kb_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_key
        )
        self._phys_sub = (
            omni.physx.get_physx_interface()
            .subscribe_physics_step_events(self._on_step)
        )
        print(">>> Isaac 키보드 운전 시작. 뷰포트를 클릭하고 W/A/S/D 또는 방향키.")
        print(">>> 중지:  key_teleop.stop()")

    def _on_key(self, event, *args):
        K = carb.input.KeyboardInput
        et = event.type
        if et == carb.input.KeyboardEventType.KEY_PRESS:
            self.pressed.add(event.input)
        elif et == carb.input.KeyboardEventType.KEY_RELEASE:
            self.pressed.discard(event.input)
        return True

    def _on_step(self, dt):
        K = carb.input.KeyboardInput
        lin = ang = 0.0
        if K.UP in self.pressed or K.W in self.pressed:
            lin += LIN_SPEED
        if K.DOWN in self.pressed or K.S in self.pressed:
            lin -= LIN_SPEED
        if K.LEFT in self.pressed or K.A in self.pressed:
            ang += ANG_SPEED
        if K.RIGHT in self.pressed or K.D in self.pressed:
            ang -= ANG_SPEED
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.pub.publish(t)

    def stop(self):
        try:
            if self._kb_sub is not None:
                self._input.unsubscribe_to_keyboard_events(
                    self._keyboard, self._kb_sub
                )
        except Exception:
            pass
        self._kb_sub = None
        if self._phys_sub is not None:
            try:
                self._phys_sub.unsubscribe()
            except Exception:
                pass
        self._phys_sub = None
        try:
            self.pub.publish(Twist())  # 정지
            self.node.destroy_node()
        except Exception:
            pass
        print(">>> Isaac 키보드 운전 중지")


# --------------------------- 부트스트랩 ---------------------------
import builtins
_reg = getattr(builtins, "_TESLA_KEY_TELEOP", [])
for _old in _reg:
    try:
        _old.stop()
    except Exception:
        pass
_reg.clear()

key_teleop = IsaacKeyTeleop()
_reg.append(key_teleop)
builtins._TESLA_KEY_TELEOP = _reg
