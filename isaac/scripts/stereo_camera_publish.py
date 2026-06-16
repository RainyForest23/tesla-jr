"""
Tesla Jr. - Stereo Camera ROS2 publisher (OmniGraph 기반)
=================================================================
front_hawk 스테레오 카메라(좌/우)의 RGB 이미지 + camera_info 를
ROS2 로 발행한다. Isaac 표준 ROS2CameraHelper 사용 (렌더링 데이터에
효율적). SHM 우회(fastdds.xml) 적용돼 있어 호스트까지 전달됨.

발행 토픽:
  /stereo/left/image_raw      sensor_msgs/Image
  /stereo/left/camera_info    sensor_msgs/CameraInfo
  /stereo/right/image_raw     sensor_msgs/Image
  /stereo/right/camera_info   sensor_msgs/CameraInfo

사용법 (Isaac Sim Script Editor):
  exec(open("/workspace/isaac/scripts/stereo_camera_publish.py").read())
  # -> Play 를 누르면 발행 시작 (이미 Play 중이면 Stop->Play 한번 권장)

주의:
  - 로봇이 /World/Carter 에 스폰돼 있어야 함 (spawn_robot.py).
  - 해상도가 높으면 렌더 부하 큼. 기본 640x480.
"""

import omni.usd
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets

# ----------------------------- 설정 -----------------------------
GRAPH_PATH = "/World/StereoCameraGraph"
WIDTH = 640
HEIGHT = 480

# (라벨, 카메라 prim 경로, 토픽 네임스페이스, frame_id)
CAMERAS = [
    ("left",  "/World/Carter/chassis_link/front_hawk/left/camera_left",
     "/stereo/left",  "stereo_left"),
    ("right", "/World/Carter/chassis_link/front_hawk/right/camera_right",
     "/stereo/right", "stereo_right"),
]
# ----------------------------------------------------------------


def build_graph():
    stage = omni.usd.get_context().get_stage()

    # 기존 그래프 있으면 제거 (재실행 대비)
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)
        print(f"기존 그래프 제거: {GRAPH_PATH}")

    keys = og.Controller.Keys
    create_nodes = [("OnTick", "omni.graph.action.OnPlaybackTick")]
    set_values = []
    connections = []

    for label, cam_path, ns, frame_id in CAMERAS:
        rp = f"createRP_{label}"
        rgb = f"camHelper_{label}_rgb"
        info = f"camHelper_{label}_info"

        create_nodes += [
            (rp,   "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
            (rgb,  "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            (info, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ]
        set_values += [
            (f"{rp}.inputs:width", WIDTH),
            (f"{rp}.inputs:height", HEIGHT),
            (f"{rgb}.inputs:topicName", f"{ns}/image_raw"),
            (f"{rgb}.inputs:frameId", frame_id),
            (f"{rgb}.inputs:type", "rgb"),
            (f"{info}.inputs:topicName", f"{ns}/camera_info"),
            (f"{info}.inputs:frameId", frame_id),
            (f"{info}.inputs:type", "camera_info"),
        ]
        connections += [
            ("OnTick.outputs:tick", f"{rp}.inputs:execIn"),
            (f"{rp}.outputs:execOut", f"{rgb}.inputs:execIn"),
            (f"{rp}.outputs:execOut", f"{info}.inputs:execIn"),
            (f"{rp}.outputs:renderProductPath", f"{rgb}.inputs:renderProductPath"),
            (f"{rp}.outputs:renderProductPath", f"{info}.inputs:renderProductPath"),
        ]

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: create_nodes,
            keys.SET_VALUES: set_values,
            keys.CONNECT: connections,
        },
    )
    print(f"그래프 생성: {GRAPH_PATH}")

    # 카메라 prim 을 각 render product 의 타겟으로 설정 (relationship)
    for label, cam_path, ns, frame_id in CAMERAS:
        rp_node_path = f"{GRAPH_PATH}/createRP_{label}"
        if not stage.GetPrimAtPath(cam_path).IsValid():
            print(f"!!! 카메라 prim 없음: {cam_path}")
            continue
        set_targets(
            prim=stage.GetPrimAtPath(rp_node_path),
            attribute="inputs:cameraPrim",
            target_prim_paths=[cam_path],
        )
        print(f"  {label} 카메라 연결: {cam_path}")

    print(">>> 스테레오 그래프 준비 완료. Play (또는 Stop->Play) 하면 발행 시작.")
    print(">>> 토픽: /stereo/left/image_raw, /stereo/right/image_raw (+ camera_info)")


build_graph()
