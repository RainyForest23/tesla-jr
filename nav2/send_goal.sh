#!/usr/bin/env bash
# Tesla Jr. - Nav2 목표 전송 (호스트에서 실행)
# =====================================================================
# 내가(Claude) 검증할 때 쓰던 것과 동일한 방식: /goal_pose 에 PoseStamped 발행.
# Nav2(bt_navigator)가 이를 받아 경로계획+주행 시작.
#
# 사용법:
#   nav2/send_goal.sh <x> <y> [yaw_deg]
#   예) nav2/send_goal.sh 2.6 4.0
#       nav2/send_goal.sh 6 0 90
#
# 전제: Nav2 실행 중(run_nav2.sh), 호스트 .bashrc 에 FASTRTPS 프로파일 등록됨.
set -e

X="${1:?사용법: send_goal.sh <x> <y> [yaw_deg]}"
Y="${2:?사용법: send_goal.sh <x> <y> [yaw_deg]}"
YAW_DEG="${3:-0}"

# frame 은 odom (지도 없이 odom 프레임 내비). yaw(deg)->쿼터니언(z,w).
# Ackermann 은 yaw_goal_tolerance 가 커서 최종 방향은 사실상 무시되지만 형식상 채움.
HALF=$(python3 -c "import math;print(math.radians($YAW_DEG)/2)")
QZ=$(python3 -c "import math;print(math.sin($HALF))")
QW=$(python3 -c "import math;print(math.cos($HALF))")

source /opt/ros/humble/setup.bash 2>/dev/null || true

echo ">>> 목표 전송: x=$X y=$Y yaw=${YAW_DEG}° (frame=odom)"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: $X, y: $Y, z: 0.0}, \
    orientation: {z: $QZ, w: $QW}}}"
echo ">>> 전송 완료. RViz 에서 경로(/plan)와 주행 확인."
