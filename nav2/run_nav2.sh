#!/bin/bash
# Tesla Jr. - Nav2 주행 스택 실행 (지도 없이 odom 프레임)
# 호스트 터미널에서 직접 실행. Isaac Sim 이 Play 중이고 launch_all + bridge 가
# 떠 있어야 함 (/odom /tf /stereo/left/points /cmd_vel).
#
# Nav2 cmd_vel 흐름: controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
# 우리 로봇(teleop_cmdvel 그래프)이 /cmd_vel 구독 -> 그대로 연결됨.

source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/Desktop/git_teslaJR/docker/fastdds.xml"

ros2 launch nav2_bringup navigation_launch.py \
  params_file:="$HOME/Desktop/git_teslaJR/nav2/teslajr_nav2.yaml" \
  use_sim_time:=false
