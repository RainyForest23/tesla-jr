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
from pxr import UsdGeom, Gf, Sdf

# ----------------------------- 설정 -----------------------------
ROBOT_PRIM_PATH = "/World/Carter"  # spawn_robot.py 가 스폰하는 경로. 다르면 수정.
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
# ----------------------------------------------------------------


class IsaacStateBridge(Node):
    def __init__(self):
        super().__init__("isaac_state_bridge")

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        self._stage = omni.usd.get_context().get_stage()
        self._robot_path = Sdf.Path(ROBOT_PRIM_PATH)
        self._xform_cache = UsdGeom.XformCache()

        self._sim_time = 0.0
        self._prev_pos = None       # np.array([x, y, z])
        self._prev_yaw = None
        self._warned_missing = False

        # 물리 스텝마다 콜백 (메인 sim 스레드에서 실행됨)
        self._phys_sub = (
            omni.physx.get_physx_interface()
            .subscribe_physics_step_events(self._on_physics_step)
        )
        self.get_logger().info(
            f"IsaacStateBridge 시작. 로봇 prim = {ROBOT_PRIM_PATH}"
        )

    # ---------- 매 물리 스텝 ----------
    def _on_physics_step(self, dt: float):
        self._sim_time += dt
        stamp = self._make_stamp(self._sim_time)

        # /clock
        clk = Clock()
        clk.clock = stamp
        self.clock_pub.publish(clk)

        # 로봇 포즈 읽기
        prim = self._stage.GetPrimAtPath(self._robot_path)
        if not prim or not prim.IsValid():
            if not self._warned_missing:
                self.get_logger().warn(
                    f"prim 을 찾을 수 없음: {ROBOT_PRIM_PATH} "
                    f"(ROBOT_PRIM_PATH 수정 필요)"
                )
                self._warned_missing = True
            return

        self._xform_cache.Clear()
        m = self._xform_cache.GetLocalToWorldTransform(prim)
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()  # Gf.Quatd
        pos = np.array([t[0], t[1], t[2]], dtype=float)
        qr = q.GetReal()
        qi = q.GetImaginary()
        quat = (qi[0], qi[1], qi[2], qr)  # (x, y, z, w)
        yaw = self._yaw_from_quat(quat)

        # 유한차분 속도
        if self._prev_pos is not None and dt > 1e-6:
            vx_w = (pos[0] - self._prev_pos[0]) / dt
            vy_w = (pos[1] - self._prev_pos[1]) / dt
            # 월드 속도 -> 로봇 기준(body) 전진/측면 속도
            v_forward = vx_w * math.cos(yaw) + vy_w * math.sin(yaw)
            v_lateral = -vx_w * math.sin(yaw) + vy_w * math.cos(yaw)
            wz = self._angle_diff(yaw, self._prev_yaw) / dt
        else:
            v_forward = v_lateral = wz = 0.0
        self._prev_pos = pos
        self._prev_yaw = yaw

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
        self.tf_pub.publish(TFMessage(transforms=[tf]))

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
