"""
Tesla Jr. - /cmd_vel 텔레옵 (OmniGraph 차동 구동)
=================================================================
ROS2 /cmd_vel (geometry_msgs/Twist) 를 구독해 Nova Carter 바퀴를 굴린다.

그래프 구성:
  OnPlaybackTick
    -> ROS2SubscribeTwist (/cmd_vel)
         linearVelocity  -> BreakVector3 -> x -> DifferentialController.linearVelocity
         angularVelocity -> BreakVector3 -> z -> DifferentialController.angularVelocity
    -> DifferentialController (wheelRadius, wheelDistance) -> velocityCommand
    -> IsaacArticulationController (joint_wheel_left/right)

사용법 (Isaac Sim Script Editor):
  exec(open("/workspace/isaac/scripts/teleop_cmdvel.py").read())
  # -> Stop -> Play

주행 (호스트 터미널):
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\
    "{linear: {x: 0.5}, angular: {z: 0.3}}"
  # 멈춤: x=0, z=0 으로 한번 더 publish
"""

import omni.usd
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets

# ----------------------------- 설정 -----------------------------
GRAPH_PATH = "/World/TeleopGraph"
ROBOT_PRIM = "/World/Carter"
WHEEL_JOINTS = ["joint_wheel_left", "joint_wheel_right"]
WHEEL_RADIUS = 0.14      # Nova Carter (m) - 필요시 튜닝
WHEEL_DISTANCE = 0.413   # 두 바퀴 간 거리 (m) - 필요시 튜닝
CMD_VEL_TOPIC = "cmd_vel"
# ----------------------------------------------------------------


def build_graph():
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)
        print(f"기존 그래프 제거: {GRAPH_PATH}")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("SubTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("BreakLin", "omni.graph.nodes.BreakVector3"),
                ("BreakAng", "omni.graph.nodes.BreakVector3"),
                ("DiffCtrl", "omni.isaac.wheeled_robots.DifferentialController"),
                ("ArtCtrl", "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubTwist.inputs:topicName", CMD_VEL_TOPIC),
                ("DiffCtrl.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffCtrl.inputs:wheelDistance", WHEEL_DISTANCE),
                ("ArtCtrl.inputs:jointNames", WHEEL_JOINTS),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "DiffCtrl.inputs:execIn"),
                ("OnTick.outputs:tick", "ArtCtrl.inputs:execIn"),
                ("SubTwist.outputs:linearVelocity", "BreakLin.inputs:tuple"),
                ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
                ("BreakLin.outputs:x", "DiffCtrl.inputs:linearVelocity"),
                ("BreakAng.outputs:z", "DiffCtrl.inputs:angularVelocity"),
                ("DiffCtrl.outputs:velocityCommand", "ArtCtrl.inputs:velocityCommand"),
            ],
        },
    )
    print(f"그래프 생성: {GRAPH_PATH}")

    # 로봇 articulation 을 ArticulationController 타겟으로 연결
    set_targets(
        prim=stage.GetPrimAtPath(f"{GRAPH_PATH}/ArtCtrl"),
        attribute="inputs:targetPrim",
        target_prim_paths=[ROBOT_PRIM],
    )
    print(f"  articulation 연결: {ROBOT_PRIM}, joints={WHEEL_JOINTS}")
    print(">>> 텔레옵 준비 완료. Stop -> Play 후 /cmd_vel 로 주행.")
    print('>>> 예: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '
          '"{linear: {x: 0.5}, angular: {z: 0.3}}"')


build_graph()
