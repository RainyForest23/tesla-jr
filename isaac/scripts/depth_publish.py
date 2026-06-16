"""
Tesla Jr. - Depth + PointCloud 발행 (OmniGraph)
=================================================================
전방 카메라(front_hawk left)에서 Isaac 렌더러의 depth 를 뽑아
ROS2 로 발행한다. Nav2 costmap 입력으로 사용.

  type="depth"      -> sensor_msgs/Image     (32FC1 깊이 이미지)
  type="depth_pcl"  -> sensor_msgs/PointCloud2 (3D 포인트클라우드)

발행 토픽:
  /stereo/left/depth     깊이 이미지
  /stereo/left/points    포인트클라우드 (costmap 입력)

사용법 (Isaac Sim Script Editor):
  exec(open("/workspace/isaac/scripts/depth_publish.py").read())
  # -> Stop -> Play

참고: Isaac ground-truth depth (1단계). 추후 stereo matching 으로 교체 예정.
      포인트클라우드 frame_id = stereo_left (카메라 TF 필요 -> 다음 단계).
"""

import omni.usd
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets

# ----------------------------- 설정 -----------------------------
GRAPH_PATH = "/World/DepthGraph"
CAMERA_PRIM = "/World/Carter/chassis_link/front_hawk/left/camera_left"
FRAME_ID = "stereo_left"
WIDTH = 640
HEIGHT = 480
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
                ("createRP", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("depthImg", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("depthPcl", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("createRP.inputs:width", WIDTH),
                ("createRP.inputs:height", HEIGHT),
                ("depthImg.inputs:topicName", "/stereo/left/depth"),
                ("depthImg.inputs:frameId", FRAME_ID),
                ("depthImg.inputs:type", "depth"),
                ("depthImg.inputs:useSystemTime", True),
                ("depthPcl.inputs:topicName", "/stereo/left/points"),
                ("depthPcl.inputs:frameId", FRAME_ID),
                ("depthPcl.inputs:type", "depth_pcl"),
                ("depthPcl.inputs:useSystemTime", True),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createRP.inputs:execIn"),
                ("createRP.outputs:execOut", "depthImg.inputs:execIn"),
                ("createRP.outputs:execOut", "depthPcl.inputs:execIn"),
                ("createRP.outputs:renderProductPath", "depthImg.inputs:renderProductPath"),
                ("createRP.outputs:renderProductPath", "depthPcl.inputs:renderProductPath"),
            ],
        },
    )
    print(f"그래프 생성: {GRAPH_PATH}")

    if not stage.GetPrimAtPath(CAMERA_PRIM).IsValid():
        print(f"!!! 카메라 prim 없음: {CAMERA_PRIM}")
        return
    set_targets(
        prim=stage.GetPrimAtPath(f"{GRAPH_PATH}/createRP"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[CAMERA_PRIM],
    )
    print(f"  카메라 연결: {CAMERA_PRIM}")
    print(">>> depth 그래프 준비 완료. Stop -> Play 하면 발행 시작.")
    print(">>> 토픽: /stereo/left/depth (이미지), /stereo/left/points (포인트클라우드)")


build_graph()
