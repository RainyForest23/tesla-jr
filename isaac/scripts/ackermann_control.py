"""
Tesla Jr. - T870 Ackermann 컨트롤러 (Python)
=================================================================
/cmd_vel (geometry_msgs/Twist) 를 받아 Ackermann 조향/구동으로 변환:
  조향각 steer = atan(wheelBase * w / v)   (앞 조향 조인트 위치 타깃)
  바퀴속도 omega = v / wheelRadius          (뒤 구동 조인트 속도 타깃)
dynamic_control 로 조인트 드라이브 게인 설정 + 타깃 적용.

조인트(DOF):
  0 front_left_steer  1 front_right_steer  (조향, 위치 드라이브)
  2 rear_left_wheel   3 rear_right_wheel   (구동, 속도 드라이브)
  4 front_left_wheel  5 front_right_wheel  (앞 rolling, 자유)

사용법 (Isaac Sim Script Editor, Play 중):
  exec(open("/workspace/isaac/scripts/ackermann_control.py").read())
주행: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear:{x:0.5}, angular:{z:0.3}}"
중지: ack.stop()
"""

import math
import omni.physx
import omni.usd
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from omni.isaac.dynamic_control import _dynamic_control
from pxr import UsdGeom, Sdf, Gf

ART_PATH = "/World/t870/base_link"
WHEEL_BASE = 0.65
WHEEL_RADIUS = 0.12
MAX_STEER = 0.6          # rad
MIN_SPEED_FOR_STEER = 0.05

# 가상 범퍼 (후면 카메라 없음 보완): 명령했는데 안 움직이면 그 방향에 장애물로 추론
STUCK_CMD_THRESH = 0.05     # 이 이상 속도 명령했는데
STUCK_SPEED_THRESH = 0.03   # 실제 속도가 이 미만이고
STUCK_TIME = 1.0            # 이 시간(초) 지속되면 stuck 판정
BUMP_DIST = 0.7            # 진행방향 접촉 추정 거리 (base 중심 기준)
BUMP_Z = 0.5              # 마킹 높이 (costmap min_obstacle_height 0.35 안에 들게)

STEER_JOINTS = ["front_left_steer_joint", "front_right_steer_joint"]
DRIVE_JOINTS = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
ROLL_JOINTS = ["front_left_wheel_joint", "front_right_wheel_joint"]

# odom/tf (bridge 통합 - dynamic_control 충돌 방지를 위해 한 노드에서 처리)
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
CAMERA_FRAME = "stereo_left"
CAMERA_PRIM_PATH = "/World/t870/front_camera_mount/front_cam"


class AckermannControl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("t870_ackermann")
        self.v = 0.0
        self.w = 0.0
        self.node.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        # odom/tf/clock 발행 (이 노드가 dc 핸들을 단독 소유 -> bridge 별도 실행 불필요)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.node.create_publisher(TFMessage, "/tf", 10)
        self.clock_pub = self.node.create_publisher(Clock, "/clock", 10)
        # 가상 범퍼: stuck 으로 추론한 장애물 포인트 발행 (costmap 소스)
        self.bump_pub = self.node.create_publisher(PointCloud2, "/virtual_bumper/points", 10)
        self.bump_points = []
        self.stuck_time = 0.0
        # 전용 executor (전역 default executor/다른 spinner 와 충돌 방지)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)

        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.art = None
        self.root_body = None
        self.dofs = {}
        self._configured = False
        self._cam_trans = None
        self._cam_quat = None
        self._compute_camera_static_tf()

        self._phys_sub = (
            omni.physx.get_physx_interface()
            .subscribe_physics_step_events(self._on_step)
        )
        print(">>> Ackermann 컨트롤러 시작. /cmd_vel 로 주행.")
        print(">>> 중지: ack.stop()")

    def _cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _acquire(self):
        self.art = self.dc.get_articulation(ART_PATH)
        if self.art == 0:
            return False
        for name in STEER_JOINTS + DRIVE_JOINTS + ROLL_JOINTS:
            self.dofs[name] = self.dc.find_articulation_dof(self.art, name)
        # odom 용 루트 바디 (동일 articulation -> 별도 get_rigid_body 호출 불필요)
        self.root_body = self.dc.get_articulation_root_body(self.art)
        return True

    def _compute_camera_static_tf(self):
        """base_link 기준 카메라 상대 변환 1회 계산 (+optical 180°X 보정)."""
        try:
            stage = omni.usd.get_context().get_stage()
            base = stage.GetPrimAtPath(Sdf.Path(ART_PATH))
            cam = stage.GetPrimAtPath(Sdf.Path(CAMERA_PRIM_PATH))
            if not base.IsValid() or not cam.IsValid():
                return
            xc = UsdGeom.XformCache()
            rel = xc.GetLocalToWorldTransform(cam) * xc.GetLocalToWorldTransform(base).GetInverse()
            optical = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0))
            rel = optical * rel
            t = rel.ExtractTranslation()
            q = rel.ExtractRotationQuat()
            qi, qr = q.GetImaginary(), q.GetReal()
            self._cam_trans = (t[0], t[1], t[2])
            self._cam_quat = (qi[0], qi[1], qi[2], qr)
        except Exception as e:
            print(f"카메라 정적 TF 계산 예외: {e}")

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _publish_odom(self, stamp):
        if self.root_body is None or self.root_body == 0:
            return
        pose = self.dc.get_rigid_body_pose(self.root_body)
        lin = self.dc.get_rigid_body_linear_velocity(self.root_body)
        ang = self.dc.get_rigid_body_angular_velocity(self.root_body)
        yaw = self._yaw(pose.r)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = float(pose.p.x)
        odom.pose.pose.position.y = float(pose.p.y)
        odom.pose.pose.position.z = float(pose.p.z)
        odom.pose.pose.orientation.x = float(pose.r.x)
        odom.pose.pose.orientation.y = float(pose.r.y)
        odom.pose.pose.orientation.z = float(pose.r.z)
        odom.pose.pose.orientation.w = float(pose.r.w)
        odom.twist.twist.linear.x = float(lin.x * math.cos(yaw) + lin.y * math.sin(yaw))
        odom.twist.twist.linear.y = float(-lin.x * math.sin(yaw) + lin.y * math.cos(yaw))
        odom.twist.twist.angular.z = float(ang.z)
        self.odom_pub.publish(odom)

        tfs = []
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = float(pose.p.x)
        tf.transform.translation.y = float(pose.p.y)
        tf.transform.translation.z = float(pose.p.z)
        tf.transform.rotation = odom.pose.pose.orientation
        tfs.append(tf)
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
            tfs.append(cam_tf)
        self.tf_pub.publish(TFMessage(transforms=tfs))

    def _add_bump(self, pose, v):
        """stuck 지점의 진행방향 끝에 가상 장애물 점들을 추가 (측면으로 짧은 벽)."""
        yaw = self._yaw(pose.r)
        fx, fy = math.cos(yaw), math.sin(yaw)
        s = 1.0 if v > 0 else -1.0          # 전진이면 앞, 후진이면 뒤
        cx = pose.p.x + s * fx * BUMP_DIST
        cy = pose.p.y + s * fy * BUMP_DIST
        lxu, lyu = -fy, fx                  # 진행방향에 수직(측면)
        for d in (-0.4, -0.2, 0.0, 0.2, 0.4):
            self.bump_points.append((cx + lxu * d, cy + lyu * d, BUMP_Z))
        if len(self.bump_points) > 800:
            self.bump_points = self.bump_points[-800:]
        print(f"[가상범퍼] stuck 감지 -> 장애물 마킹 @({cx:.2f},{cy:.2f}) "
              f"{'전진' if v > 0 else '후진'}방향")

    def _publish_bump(self, stamp):
        n = len(self.bump_points)
        if n == 0:
            return
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = ODOM_FRAME
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.is_dense = True
        msg.data = np.array(self.bump_points, dtype=np.float32).tobytes()
        self.bump_pub.publish(msg)

    def _configure_drives(self):
        # 드라이브 게인 설정: 조향=위치(stiffness 큼), 구동=속도(stiffness 0, damping 큼),
        # 앞 rolling=자유(거의 무저항)
        props = self.dc.get_articulation_dof_properties(self.art)
        names = [self.dc.get_dof_name(self.dc.get_articulation_dof(self.art, i))
                 for i in range(self.dc.get_articulation_dof_count(self.art))]
        for i, nm in enumerate(names):
            # 위치제어=stiffness>0, 속도제어=stiffness 0 + damping.
            # 조향 링크 관성이 작아 FORCE 모드 고stiffness는 불안정 -> ACCELERATION 모드
            # (게인이 관성과 무관). FORCE=0, ACCELERATION=1.
            if nm in STEER_JOINTS:
                props["driveMode"][i] = getattr(_dynamic_control, "DRIVE_ACCELERATION", 1)
                props["stiffness"][i] = 500.0
                props["damping"][i] = 50.0
                props["maxEffort"][i] = 1.0e6
            elif nm in DRIVE_JOINTS:
                props["driveMode"][i] = _dynamic_control.DRIVE_FORCE
                props["stiffness"][i] = 0.0
                props["damping"][i] = 1.0e4
                props["maxEffort"][i] = 1.0e5
            elif nm in ROLL_JOINTS:
                # 앞바퀴는 완전 자유 회전이어야 함 (damping>0 이면 브레이크처럼
                # 굴러가는 걸 막아 전진을 방해). stiffness/damping 모두 0.
                props["driveMode"][i] = _dynamic_control.DRIVE_FORCE
                props["stiffness"][i] = 0.0
                props["damping"][i] = 0.0
                props["maxEffort"][i] = 0.0
        self.dc.set_articulation_dof_properties(self.art, props)
        print("드라이브 게인 설정 완료")

    def _on_step(self, dt):
        # /cmd_vel 콜백 처리 (전용 executor, 단일 스레드)
        try:
            self._exec.spin_once(timeout_sec=0.0)
        except Exception:
            pass
        if self.art is None or self.art == 0:
            if not self._acquire():
                return
        if not self._configured:
            try:
                self._configure_drives()
                self._configured = True
            except Exception as e:
                print(f"드라이브 설정 예외: {e}")
                return

        self.dc.wake_up_articulation(self.art)
        v, w = self.v, self.w

        # --- 가상 범퍼: 명령했는데 실제로 안 움직이면 그 방향에 장애물 추론 ---
        pose = None
        speed = 1.0   # 모르면 stuck 판정 안 함
        if self.root_body and self.root_body != 0:
            pose = self.dc.get_rigid_body_pose(self.root_body)
            lv = self.dc.get_rigid_body_linear_velocity(self.root_body)
            speed = math.hypot(lv.x, lv.y)
        force_stop = False
        if abs(v) > STUCK_CMD_THRESH and speed < STUCK_SPEED_THRESH:
            self.stuck_time += dt
            if self.stuck_time > STUCK_TIME and pose is not None:
                self._add_bump(pose, v)     # 장애물 마킹
                force_stop = True           # 갈리지 않게 정지
                self.stuck_time = 0.0
        else:
            self.stuck_time = 0.0

        # Ackermann 조향각 (정지 시 0)
        if force_stop:
            steer, omega = 0.0, 0.0
        else:
            if abs(v) > MIN_SPEED_FOR_STEER:
                steer = math.atan(WHEEL_BASE * w / v)
            else:
                steer = 0.0
            steer = max(-MAX_STEER, min(MAX_STEER, steer))
            # 바퀴 축이 +Y -> omega>0 이 +X 전진 (물리적으로 맞음)
            omega = v / WHEEL_RADIUS

        for nm in STEER_JOINTS:
            self.dc.set_dof_position_target(self.dofs[nm], steer)
        for nm in DRIVE_JOINTS:
            self.dc.set_dof_velocity_target(self.dofs[nm], omega)

        # odom/tf/clock 발행 (wall-clock, 카메라/RViz 와 시간 일치)
        stamp = self.node.get_clock().now().to_msg()
        clk = Clock()
        clk.clock = stamp
        self.clock_pub.publish(clk)
        self._publish_odom(stamp)
        self._publish_bump(stamp)

    def stop(self):
        if self._phys_sub is not None:
            try:
                self._phys_sub.unsubscribe()
            except Exception:
                pass
        self._phys_sub = None
        # 정지
        try:
            if self.art and self.art != 0:
                for nm in DRIVE_JOINTS:
                    self.dc.set_dof_velocity_target(self.dofs[nm], 0.0)
        except Exception:
            pass
        try:
            self._exec.remove_node(self.node)
            self._exec.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        print(">>> Ackermann 컨트롤러 중지")


# ---- 부트스트랩 (이전 인스턴스 정리) ----
import builtins
_reg = getattr(builtins, "_TESLA_ACK", [])
for _o in _reg:
    try:
        _o.stop()
    except Exception:
        pass
_reg.clear()
ack = AckermannControl()
_reg.append(ack)
builtins._TESLA_ACK = _reg
