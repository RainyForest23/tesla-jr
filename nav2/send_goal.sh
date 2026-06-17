#!/usr/bin/env bash
# Tesla Jr. - Nav2 목표 전송 (호스트에서 실행)
# =====================================================================
# 목표를 JSON 으로 출력하고, 모드에 따라 발행:
#   pose (기본) : /goal_pose 에 PoseStamped 직접 발행 (Nav2 bt_navigator 직접)
#   json        : /goal_json 에 JSON(String) 발행 (json_goal_node 경유 = VLM 이 쓸 경로)
#   both        : 둘 다
#
# 사용법:
#   nav2/send_goal.sh <x> <y> [yaw_deg] [mode]
#   예) nav2/send_goal.sh 12 0
#       nav2/send_goal.sh 6 0 90 json     # json_goal_node 실행 중이어야 함
#       nav2/send_goal.sh 2.6 4 0 both
#
# 전제: Nav2 실행 중(run_nav2.sh), .bashrc 에 FASTRTPS 프로파일 등록.
#       json 모드는 json_goal_node.py 도 실행 중이어야 함.
set -e

X="${1:?사용법: send_goal.sh <x> <y> [yaw_deg] [pose|json|both]}"
Y="${2:?사용법: send_goal.sh <x> <y> [yaw_deg] [pose|json|both]}"
YAW_DEG="${3:-0}"
MODE="${4:-pose}"

# yaw(deg) -> 쿼터니언(z,w). (Ackermann 은 yaw_goal_tolerance 커서 최종방향 거의 무시)
HALF=$(python3 -c "import math;print(math.radians($YAW_DEG)/2)")
QZ=$(python3 -c "import math;print(math.sin($HALF))")
QW=$(python3 -c "import math;print(math.cos($HALF))")

# JSON 표현 (VLM 출력과 동일 포맷)
JSON="{\"x\": $X, \"y\": $Y, \"yaw\": $YAW_DEG}"

source /opt/ros/humble/setup.bash 2>/dev/null || true

echo ">>> 목표: x=$X y=$Y yaw=${YAW_DEG}°  (mode=$MODE)"
echo ">>> JSON: $JSON"

if [ "$MODE" = "pose" ] || [ "$MODE" = "both" ]; then
  echo ">>> /goal_pose 발행 (PoseStamped, frame=odom)"
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'odom'}, pose: {position: {x: $X, y: $Y, z: 0.0}, \
      orientation: {z: $QZ, w: $QW}}}"
fi

if [ "$MODE" = "json" ] || [ "$MODE" = "both" ]; then
  echo ">>> /goal_json 발행 (String, json_goal_node 경유)"
  ros2 topic pub --once /goal_json std_msgs/msg/String "{data: '$JSON'}"
fi

echo ">>> 전송 완료. RViz 에서 경로(/plan)와 주행 확인."
