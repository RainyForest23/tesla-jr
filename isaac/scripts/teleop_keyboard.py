#!/usr/bin/env python3
"""
Tesla Jr. - 방향키 키보드 텔레옵 (호스트에서 실행)
=================================================================
방향키로 /cmd_vel 을 발행해 Isaac Sim 의 Carter 를 운전한다.
누르고 있는 동안 이동, 떼면 ~0.3초 후 자동 정지 (키 자동반복 이용).

  ↑ : 전진      ↓ : 후진
  ← : 좌회전    → : 우회전
  Space : 즉시 정지
  + / - : 속도 증감
  q / Ctrl-C : 종료

실행 (호스트 터미널, 직접 실행):
  python3 ~/Desktop/git_teslaJR/isaac/scripts/teleop_keyboard.py

주의: 이 스크립트는 자기 터미널의 키 입력을 받으므로 반드시
      사용자가 직접 터미널에서 실행해야 한다 (Claude 가 대신 못 침).
      ~/.bashrc 에 FASTRTPS_DEFAULT_PROFILES_FILE 설정돼 있어야 데이터 전달됨.
"""

import sys
import time
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LIN_STEP = 0.4      # 기본 선속도 (m/s)
ANG_STEP = 0.8      # 기본 각속도 (rad/s)
HOLD_TIME = 0.3     # 마지막 키 입력 후 이 시간(s) 지나면 정지
PUB_RATE = 20.0     # 발행 주기 (Hz)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.lin_scale = 1.0
        self.ang_scale = 1.0

    def make_twist(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        return t


def read_key(timeout):
    """timeout 초 동안 키 1개 읽기. 방향키는 ESC 시퀀스(\\x1b[A 등)."""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":  # ESC 시퀀스 (방향키)
        seq = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.01)[0] else ""
        return "\x1b" + seq
    return ch


def main():
    rclpy.init()
    node = KeyboardTeleop()

    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    target = (0.0, 0.0)
    active_until = 0.0
    period = 1.0 / PUB_RATE

    print(__doc__)
    print(f"[속도] linear={LIN_STEP:.2f} m/s  angular={ANG_STEP:.2f} rad/s")
    print("운전 시작. 방향키를 누르세요...")

    try:
        while rclpy.ok():
            key = read_key(period)
            now = time.time()

            if key is not None:
                if key == "\x1b[A":      # ↑
                    target = (LIN_STEP * node.lin_scale, 0.0); active_until = now + HOLD_TIME
                elif key == "\x1b[B":    # ↓
                    target = (-LIN_STEP * node.lin_scale, 0.0); active_until = now + HOLD_TIME
                elif key == "\x1b[D":    # ←
                    target = (0.0, ANG_STEP * node.ang_scale); active_until = now + HOLD_TIME
                elif key == "\x1b[C":    # →
                    target = (0.0, -ANG_STEP * node.ang_scale); active_until = now + HOLD_TIME
                elif key == " ":         # 정지
                    target = (0.0, 0.0); active_until = 0.0
                elif key in ("+", "="):
                    node.lin_scale = min(node.lin_scale + 0.2, 3.0)
                    node.ang_scale = min(node.ang_scale + 0.2, 3.0)
                    print(f"[속도] x{node.lin_scale:.1f}")
                elif key == "-":
                    node.lin_scale = max(node.lin_scale - 0.2, 0.2)
                    node.ang_scale = max(node.ang_scale - 0.2, 0.2)
                    print(f"[속도] x{node.lin_scale:.1f}")
                elif key in ("q", "\x03"):  # q or Ctrl-C
                    break

            # 발행: 활성 시간 안이면 target, 아니면 정지
            if now < active_until:
                lin, ang = target
            else:
                lin, ang = 0.0, 0.0
            node.pub.publish(node.make_twist(lin, ang))
    finally:
        # 종료 시 정지 명령
        node.pub.publish(node.make_twist(0.0, 0.0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        node.destroy_node()
        rclpy.shutdown()
        print("\n텔레옵 종료.")


if __name__ == "__main__":
    main()
