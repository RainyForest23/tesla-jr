"""
Tesla Jr. - T870 전방 카메라 + depth/포인트클라우드 publish
=================================================================
T870 의 front_camera_mount 에 전방(+X) 카메라를 생성하고, RGB + camera_info +
depth 포인트클라우드를 ROS2 로 발행. Nav2 costmap 입력.

토픽(Carter 때와 동일 재사용 -> Nav2/RViz 설정 불변):
  /stereo/left/image_raw    RGB
  /stereo/left/camera_info  intrinsics
  /stereo/left/points       포인트클라우드 (costmap 입력)

사용법 (Isaac Sim Script Editor):
  exec(open("/workspace/isaac/scripts/t870_camera.py").read())
  # -> Stop -> Play
"""

import omni.usd
import omni.graph.core as og
from pxr import UsdGeom, Gf, Sdf
from omni.isaac.core.utils.prims import set_targets

MOUNT = "/World/t870/front_camera_mount"
CAM_PRIM = "/World/t870/front_camera_mount/front_cam"
GRAPH_PATH = "/World/T870CameraGraph"
FRAME_ID = "stereo_left"
WIDTH, HEIGHT = 640, 480


def create_camera():
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(Sdf.Path(MOUNT)).IsValid():
        print(f"!!! mount 없음: {MOUNT} (T870 임포트 확인)")
        return False
    if stage.GetPrimAtPath(Sdf.Path(CAM_PRIM)).IsValid():
        print("카메라 이미 있음")
        return True
    cam = UsdGeom.Camera.Define(stage, CAM_PRIM)
    # 전방(+X) 을 보도록 회전. USD 카메라는 local -Z 를 봄.
    # local -Z -> +X, local +Y -> +Z(up). (row-vector 행렬: 행=local축의 parent표현)
    m = Gf.Matrix3d(0, -1, 0,   0, 0, 1,   -1, 0, 0)
    quat = m.ExtractRotation().GetQuat()
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.AddOrientOp().Set(Gf.Quatf(quat))
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
    print(f"카메라 생성: {CAM_PRIM} (전방 +X)")
    return True


def build_graph():
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(Sdf.Path(GRAPH_PATH)).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("createRP", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("rgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("info", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("pcl", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("createRP.inputs:width", WIDTH),
                ("createRP.inputs:height", HEIGHT),
                ("rgb.inputs:topicName", "/stereo/left/image_raw"),
                ("rgb.inputs:frameId", FRAME_ID),
                ("rgb.inputs:type", "rgb"),
                ("rgb.inputs:useSystemTime", True),
                ("info.inputs:topicName", "/stereo/left/camera_info"),
                ("info.inputs:frameId", FRAME_ID),
                ("info.inputs:type", "camera_info"),
                ("info.inputs:useSystemTime", True),
                ("pcl.inputs:topicName", "/stereo/left/points"),
                ("pcl.inputs:frameId", FRAME_ID),
                ("pcl.inputs:type", "depth_pcl"),
                ("pcl.inputs:useSystemTime", True),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createRP.inputs:execIn"),
                ("createRP.outputs:execOut", "rgb.inputs:execIn"),
                ("createRP.outputs:execOut", "info.inputs:execIn"),
                ("createRP.outputs:execOut", "pcl.inputs:execIn"),
                ("createRP.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
                ("createRP.outputs:renderProductPath", "info.inputs:renderProductPath"),
                ("createRP.outputs:renderProductPath", "pcl.inputs:renderProductPath"),
            ],
        },
    )
    set_targets(
        prim=stage.GetPrimAtPath(f"{GRAPH_PATH}/createRP"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[CAM_PRIM],
    )
    print(f"카메라 그래프 생성: {GRAPH_PATH}")
    print(">>> Stop -> Play 하면 /stereo/left/{image_raw,camera_info,points} 발행")


if create_camera():
    build_graph()
