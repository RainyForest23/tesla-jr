#!/usr/bin/env python3
"""
Tesla Jr. - JSON Goal -> Nav2 목표 노드 (호스트 실행, Phase 2)
=====================================================================
JSON 형식의 목표를 받아 Nav2 의 navigate_to_pose 액션으로 전송한다.
VLM(Gemma 4) 이 내놓을 JSON 출력을 실제 주행 목표로 변환하는 '다리'.

구독:
  /goal_json  std_msgs/String
      예) {"x": 2.5, "y": 4.0}            (yaw 생략 가능, 단위 deg)
          {"x": 6.0, "y": 0.0, "yaw": 90}
동작:
  파싱 -> PoseStamped(frame=odom) -> NavigateToPose 액션 전송 (수락/완료 로깅).

실행:
  python3 nodes/json_goal_node.py
  # 다른 터미널에서 목표 주기:
  ros2 topic pub --once /goal_json std_msgs/msg/String "{data: '{\"x\":2.5,\"y\":4.0}'}"

의존: nav2_msgs (Nav2 설치 시 포함). Nav2(run_nav2.sh) 가 떠 있어야 함.
주의: 호스트 .bashrc 에 FASTRTPS_DEFAULT_PROFILES_FILE export 필요.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

GOAL_FRAME = "odom"   # 지도 없이 odom 프레임 내비 (teslajr_nav2.yaml 과 일치)


class JsonGoalNode(Node):
    def __init__(self):
        super().__init__("json_goal_node")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.sub = self.create_subscription(String, "/goal_json", self.on_json, 10)
        self.get_logger().info(
            "대기 중: /goal_json  예) {\"x\":2.5,\"y\":4.0,\"yaw\":0}")

    def on_json(self, msg):
        try:
            d = json.loads(msg.data)
            x = float(d["x"])
            y = float(d["y"])
            yaw = math.radians(float(d.get("yaw", 0.0)))
        except Exception as e:
            self.get_logger().error(f"JSON 파싱 실패: {e} / 받은값={msg.data!r}")
            return

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                "navigate_to_pose 액션 서버 없음 -> Nav2(run_nav2.sh) 실행 확인")
            return

        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = GOAL_FRAME
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = p

        self.get_logger().info(
            f"목표 전송: x={x} y={y} yaw={math.degrees(yaw):.0f}° (frame={GOAL_FRAME})")
        self.client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("목표 거부됨")
            return
        self.get_logger().info("목표 수락 -> 주행 중...")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.get_logger().info("목표 처리 완료 (결과 수신).")


def main():
    rclpy.init()
    node = JsonGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
