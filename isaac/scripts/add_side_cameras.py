"""
Tesla Jr. - T870 좌/우 스테레오 카메라 추가 (front 파이프라인 불변)
=================================================================
실제 T870 설계와 동일하게 정면+좌+우 3개 카메라 구성. front 카메라는
t870_camera.py 가 이미 /stereo/left/* 로 발행 중이므로 건드리지 않고,
이 스크립트는 좌/우 카메라만 추가한다.

마운트(URDF)는 이미 회전돼 있음:
  left_camera_mount  : yaw +90° -> 카메라가 +Y(좌) 를 봄
  right_camera_mount : yaw -90° -> 카메라가 -Y(우) 를 봄
front 와 동일한 카메라 방향 행렬(local -Z -> mount +X)을 그대로 쓰면
마운트 회전 덕에 좌/우 방향이 자동으로 맞는다.

발행 토픽:
  /left_cam/{image_raw,camera_info,points}
  /right_cam/{image_raw,camera_info,points}
정적 TF (latched /tf_static): base_link -> {left_cam, right_cam}
  (front 의 base_link->stereo_left TF 는 ackermann_control.py 가 계속 발행)

사용법 (Isaac Sim Script Editor):
  exec(open("/workspace/isaac/scripts/launch_t870.py").read())
  exec(open("/workspace/isaac/scripts/add_side_cameras.py").read())
  # -> Play(▶) -> fix_friction -> ackermann_control
"""

import builtins
import omni.usd
import omni.graph.core as og
from pxr import UsdGeom, Gf, Sdf
from omni.isaac.core.utils.prims import set_targets

BASE_FRAME = "base_link"
ART_PATH = "/World/t870/base_link"
WIDTH, HEIGHT = 640, 480

# 화각/각도 튜닝
#  FOCAL_MM: 작을수록 광각. 16mm ≈ 수평 66° (기본 aperture 20.955mm 기준).
#  yaw_deg : 마운트(±90°) 기준 카메라를 앞쪽으로 더 틀어주는 각.
#            left -30° / right +30° -> 좌우 카메라 중심이 ±60° -> front 와 합쳐
#            전방 약 180° 연속 커버 (화각 66° 라 이음새 겹침).
FOCAL_MM = 12.0   # ~수평 82° (넓게: front 와 이음새 겹치고 옆까지 커버)

CAMERAS = [
    {
        "mount": "/World/t870/left_camera_mount",
        "prim":  "/World/t870/left_camera_mount/left_cam",
        "graph": "/World/T870LeftCamGraph",
        "frame": "left_cam",
        "topic": "/left_cam",
        "yaw_deg": -20.0,   # 앞쪽 20° 틸트 (mount +90° -> 실제 +70°, 시야 +29~+111°)
    },
    {
        "mount": "/World/t870/right_camera_mount",
        "prim":  "/World/t870/right_camera_mount/right_cam",
        "graph": "/World/T870RightCamGraph",
        "frame": "right_cam",
        "topic": "/right_cam",
        "yaw_deg": +20.0,   # 실제 -70°, 시야 -29~-111° (코너 옆벽까지 보임)
    },
]

# front 와 동일: USD 카메라 local -Z 를 mount +X 로 (mount 회전이 방향 결정)
_CAM_BASE = Gf.Matrix3d(0, -1, 0,   0, 0, 1,   -1, 0, 0)


def _cam_quat(yaw_deg):
    """기본 방향(_CAM_BASE) 에 mount-local up(+Z) 기준 yaw 를 추가."""
    yaw = Gf.Matrix3d(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg))
    return (_CAM_BASE * yaw).ExtractRotation().GetQuat()


def create_camera(cfg):
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(Sdf.Path(cfg["mount"])).IsValid():
        print(f"!!! mount 없음: {cfg['mount']} (T870 임포트 확인)")
        return False
    # 재실행 시 각도/화각 갱신 위해 기존 prim 제거 후 재생성
    if stage.GetPrimAtPath(Sdf.Path(cfg["prim"])).IsValid():
        stage.RemovePrim(cfg["prim"])
    cam = UsdGeom.Camera.Define(stage, cfg["prim"])
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.AddOrientOp().Set(Gf.Quatf(_cam_quat(cfg["yaw_deg"])))
    cam.CreateFocalLengthAttr(FOCAL_MM)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
    print(f"카메라 생성: {cfg['prim']} (frame={cfg['frame']}, "
          f"yaw={cfg['yaw_deg']:+.0f}°, focal={FOCAL_MM}mm)")
    return True


def build_graph(cfg):
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(Sdf.Path(cfg["graph"])).IsValid():
        stage.RemovePrim(cfg["graph"])

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": cfg["graph"], "evaluator_name": "execution"},
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
                ("rgb.inputs:topicName", f"{cfg['topic']}/image_raw"),
                ("rgb.inputs:frameId", cfg["frame"]),
                ("rgb.inputs:type", "rgb"),
                ("rgb.inputs:useSystemTime", True),
                ("info.inputs:topicName", f"{cfg['topic']}/camera_info"),
                ("info.inputs:frameId", cfg["frame"]),
                ("info.inputs:type", "camera_info"),
                ("info.inputs:useSystemTime", True),
                ("pcl.inputs:topicName", f"{cfg['topic']}/points"),
                ("pcl.inputs:frameId", cfg["frame"]),
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
        prim=stage.GetPrimAtPath(f"{cfg['graph']}/createRP"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[cfg["prim"]],
    )
    print(f"카메라 그래프 생성: {cfg['graph']} -> {cfg['topic']}/*")


def _rel_tf(cfg):
    """base_link 기준 카메라 정적 변환 (front 와 동일: optical 180°X 보정).
    cam/base 모두 base_link 에 강체 고정이라 물리와 무관한 고정값."""
    stage = omni.usd.get_context().get_stage()
    base = stage.GetPrimAtPath(Sdf.Path(ART_PATH))
    cam = stage.GetPrimAtPath(Sdf.Path(cfg["prim"]))
    if not base.IsValid() or not cam.IsValid():
        return None
    xc = UsdGeom.XformCache()
    rel = xc.GetLocalToWorldTransform(cam) * xc.GetLocalToWorldTransform(base).GetInverse()
    optical = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0))
    rel = optical * rel
    t = rel.ExtractTranslation()
    q = rel.ExtractRotationQuat()
    qi, qr = q.GetImaginary(), q.GetReal()
    return (t[0], t[1], t[2]), (qi[0], qi[1], qi[2], qr)


def publish_static_tf(cfgs):
    """좌/우 카메라 정적 TF 를 /tf_static (latched) 로 1회 발행."""
    import rclpy
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    if not rclpy.ok():
        rclpy.init()

    # 기존 노드 정리 (재실행 대비)
    old = getattr(builtins, "_TESLA_SIDECAM", None)
    if old is not None:
        try:
            old.destroy_node()
        except Exception:
            pass

    node = rclpy.create_node("t870_side_cam_tf")
    qos = QoSProfile(
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,   # latched
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST, depth=1,
    )
    pub = node.create_publisher(TFMessage, "/tf_static", qos)

    tfs = []
    now = node.get_clock().now().to_msg()
    for cfg in cfgs:
        res = _rel_tf(cfg)
        if res is None:
            print(f"!!! TF 계산 실패(prim 없음): {cfg['frame']}")
            continue
        (tx, ty, tz), (qx, qy, qz, qw) = res
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = BASE_FRAME
        tf.child_frame_id = cfg["frame"]
        tf.transform.translation.x = float(tx)
        tf.transform.translation.y = float(ty)
        tf.transform.translation.z = float(tz)
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        tfs.append(tf)
        print(f"정적 TF: {BASE_FRAME} -> {cfg['frame']} "
              f"t=({tx:.2f},{ty:.2f},{tz:.2f})")

    if tfs:
        pub.publish(TFMessage(transforms=tfs))
    builtins._TESLA_SIDECAM = node   # latched 유지를 위해 노드 생존
    print(">>> /tf_static 발행 완료 (latched).")


# ─── 실행 ────────────────────────────────────────────────────────
_made = []
for _cfg in CAMERAS:
    if create_camera(_cfg):
        build_graph(_cfg)
        _made.append(_cfg)
publish_static_tf(_made)
print("\n>>> 좌/우 카메라 추가 완료. Stop -> Play 하면 발행 시작.")
print(">>> 토픽: /left_cam/*, /right_cam/*  (+ /tf_static 의 left_cam,right_cam)")
