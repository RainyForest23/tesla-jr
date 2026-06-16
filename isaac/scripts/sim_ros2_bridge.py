"""
Tesla Jr. - Isaac Sim -> host ROS2 state publisher (rclpy 기반)
=================================================================
검증된 rclpy 경로로 로봇 상태를 ROS2로 내보낸다.
OmniGraph 의 무거운 센서 배선 대신, 물리 스텝 콜백에서 직접 prim
월드 변환을 읽어 publish 하므로 가볍고 디버깅이 쉽다.

발행 토픽:
  /clock   rosgraph_msgs/Clock        (시뮬레이션 시간)
  /odom    nav_msgs/Odometry          (로봇 prim 월드 포즈 + 유한차분 속도)
  /tf      tf2_msgs/TFMessage         (odom -> base_link)

사용법 (Isaac Sim Script Editor):
  exec(open("/workspace/isaac/scripts/sim_ros2_bridge.py").read())
  # -> Play 를 누르면 발행 시작

중지:
  bridge.stop()

주의:
  - 씬에 Ground Plane + 로봇이 있어야 한다. ROBOT_PRIM_PATH 를 로봇 루트
    prim 경로에 맞게 수정할 것 (Stage 패널에서 확인).
  - 호스트에서 데이터 보려면 FASTRTPS_DEFAULT_PROFILES_FILE 가 설정돼 있어야 한다
    (.bashrc 에 이미 등록됨).
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import omni.usd
import omni.physx
import omni.timeline
from omni.isaac.dynamic_control import _dynamic_control
from pxr import UsdGeom, Sdf, Gf

# ----------------------------- 설정 -----------------------------
# T870: 실제 움직이는 차체 링크(base_link)를 읽음.
ROBOT_PRIM_PATH = "/World/t870/base_link"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

# 카메라 정적 TF (고정 마운트). 포인트클라우드/이미지 frame_id 와 일치시킬 것.
# t870_camera.py 가 생성하는 전방 카메라. 없으면 카메라 TF 는 건너뜀.
CAMERA_FRAME = "stereo_left"
CAMERA_PRIM_PATH = "/World/t870/front_camera_mount/front_cam"
# ----------------------------------------------------------------


class IsaacStateBridge(Node):
    def __init__(self):
        super().__init__("isaac_state_bridge")

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        # 물리 엔진의 실제 강체 pose/속도를 읽기 위한 dynamic_control 인터페이스
        # (USD XformCache 는 정적 스폰값만 반환하므로 사용 불가)
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._handle = None

        # Isaac 타임라인 시간 (카메라/clock 과 동일 소스). 직접 누적하면
        # 재실행 시 0으로 리셋돼 카메라 stamp 와 어긋나므로 타임라인을 읽는다.
        self._timeline = omni.timeline.get_timeline_interface()
        self._sim_time = 0.0
        self._warned_missing = False

        # 카메라 정적 TF (base_link -> camera) 1회 계산 (고정 마운트)
        self._cam_trans = None
        self._cam_quat = None
        self._compute_camera_static_tf()

        # 물리 스텝마다 콜백 (메인 sim 스레드에서 실행됨)
        self._phys_sub = (
            omni.physx.get_physx_interface()
            .subscribe_physics_step_events(self._on_physics_step)
        )
        self.get_logger().info(
            f"IsaacStateBridge 시작. 로봇 prim = {ROBOT_PRIM_PATH}"
        )

    def _compute_camera_static_tf(self):
        """base_link(chassis) 기준 카메라의 상대 변환을 USD 에서 1회 읽음."""
        try:
            stage = omni.usd.get_context().get_stage()
            base = stage.GetPrimAtPath(Sdf.Path(ROBOT_PRIM_PATH))
            cam = stage.GetPrimAtPath(Sdf.Path(CAMERA_PRIM_PATH))
            if not base.IsValid() or not cam.IsValid():
                self.get_logger().warn("카메라 정적 TF 계산 실패 (prim 없음)")
                return
            xc = UsdGeom.XformCache()
            m_base = xc.GetLocalToWorldTransform(base)
            m_cam = xc.GetLocalToWorldTransform(cam)
            rel = m_cam * m_base.GetInverse()  # USD 카메라 prim 을 base 프레임으로
            # optical 보정: USD 카메라(-Z 전방) -> ROS optical(+Z 전방).
            # X축 180° 회전 (Y,Z 뒤집기). 클라우드가 optical 규약이므로 일치시킴.
            optical = Gf.Matrix4d().SetRotate(
                Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)
            )
            rel = optical * rel
            t = rel.ExtractTranslation()
            q = rel.ExtractRotationQuat()
            qi, qr = q.GetImaginary(), q.GetReal()
            self._cam_trans = (t[0], t[1], t[2])
            self._cam_quat = (qi[0], qi[1], qi[2], qr)
            self.get_logger().info(
                f"카메라 정적 TF: {BASE_FRAME} -> {CAMERA_FRAME} "
                f"trans={tuple(round(v,3) for v in self._cam_trans)}"
            )
        except Exception as e:
            self.get_logger().warn(f"카메라 정적 TF 계산 예외: {e}")

    # ---------- 매 물리 스텝 ----------
    def _on_physics_step(self, dt: float):
        # 시스템(wall) 시간 사용 -> 카메라(useSystemTime=True) 및 RViz 와 시간 일치.
        stamp = self.get_clock().now().to_msg()

        # /clock
        clk = Clock()
        clk.clock = stamp
        self.clock_pub.publish(clk)

        # 강체 핸들 지연 획득 (Play 이후 물리 뷰가 있어야 유효)
        if self._handle is None:
            self._handle = self._dc.get_rigid_body(ROBOT_PRIM_PATH)
            if self._handle == _dynamic_control.INVALID_HANDLE or not self._handle:
                self._handle = None
                if not self._warned_missing:
                    self.get_logger().warn(
                        f"강체 핸들 획득 실패: {ROBOT_PRIM_PATH} "
                        f"(Play 중인지, 경로가 rigid body 인지 확인)"
                    )
                    self._warned_missing = True
                return

        # 물리 엔진의 실제 pose / 속도 읽기
        pose = self._dc.get_rigid_body_pose(self._handle)
        lin = self._dc.get_rigid_body_linear_velocity(self._handle)
        ang = self._dc.get_rigid_body_angular_velocity(self._handle)

        pos = (pose.p.x, pose.p.y, pose.p.z)
        quat = (pose.r.x, pose.r.y, pose.r.z, pose.r.w)  # (x, y, z, w)
        yaw = self._yaw_from_quat(quat)

        # 월드 선속도 -> 로봇 기준(body) 전진/측면 속도
        v_forward = lin.x * math.cos(yaw) + lin.y * math.sin(yaw)
        v_lateral = -lin.x * math.sin(yaw) + lin.y * math.cos(yaw)
        wz = ang.z

        # /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        odom.twist.twist.linear.x = float(v_forward)
        odom.twist.twist.linear.y = float(v_lateral)
        odom.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(odom)

        # /tf : odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        tf.transform.rotation.x = float(quat[0])
        tf.transform.rotation.y = float(quat[1])
        tf.transform.rotation.z = float(quat[2])
        tf.transform.rotation.w = float(quat[3])

        transforms = [tf]

        # base_link -> camera 정적 TF (포인트클라우드/이미지 배치용)
        if self._cam_trans is not None:
            cam_tf = TransformStamped()
            cam_tf.header.stamp = stamp
            cam_tf.header.frame_id = BASE_FRAME
            cam_tf.child_frame_id = CAMERA_FRAME
            cam_tf.transform.translation.x = float(self._cam_trans[0])
            cam_tf.transform.translation.y = float(self._cam_trans[1])
            cam_tf.transform.translation.z = float(self._cam_trans[2])
            cam_tf.transform.rotation.x = float(self._cam_quat[0])
            cam_tf.transform.rotation.y = float(self._cam_quat[1])
            cam_tf.transform.rotation.z = float(self._cam_quat[2])
            cam_tf.transform.rotation.w = float(self._cam_quat[3])
            transforms.append(cam_tf)

        self.tf_pub.publish(TFMessage(transforms=transforms))

    # ---------- 유틸 ----------
    @staticmethod
    def _make_stamp(sim_time: float):
        from builtin_interfaces.msg import Time as TimeMsg
        s = TimeMsg()
        s.sec = int(sim_time)
        s.nanosec = int((sim_time - int(sim_time)) * 1e9)
        return s

    @staticmethod
    def _yaw_from_quat(q):
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def stop(self):
        # 물리 콜백 구독 명시적 해제 (= None 만으로는 안 풀리는 빌드 있음)
        if self._phys_sub is not None:
            try:
                self._phys_sub.unsubscribe()
            except Exception:
                pass
        self._phys_sub = None
        try:
            self.get_logger().info("IsaacStateBridge 중지")
            self.destroy_node()
        except Exception:
            pass


# --------------------------- 부트스트랩 ---------------------------
if not rclpy.ok():
    rclpy.init()

# 전역 레지스트리로 모든 이전 인스턴스 정리 (좀비 콜백 방지)
import builtins
_reg = getattr(builtins, "_TESLA_BRIDGES", [])
for old in _reg:
    try:
        old.stop()
    except Exception:
        pass
_reg.clear()

bridge = IsaacStateBridge()
_reg.append(bridge)
builtins._TESLA_BRIDGES = _reg
print(">>> IsaacStateBridge 준비 완료. Play 를 누르면 /clock /odom /tf 발행 시작.")
print(">>> 중지하려면 Script Editor 에서:  bridge.stop()")
