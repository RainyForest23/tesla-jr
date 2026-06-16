#!/usr/bin/env python3
"""
Tesla Jr. - YOLO 객체탐지 노드 (호스트 실행, Phase 2)
=====================================================================
Isaac 카메라 이미지 토픽을 구독해 YOLO 추론 후, 박스를 그린 이미지와
탐지 결과(JSON)를 발행한다. GPU 는 호스트(RTX 3060)에서 사용.

구독:
  /stereo/left/image_raw            (기본; --image-topic 으로 변경)
발행:
  /yolo/image            sensor_msgs/Image   박스 그린 주석 이미지 (RViz 표시용)
  /yolo/detections_json  std_msgs/String     [{"name","conf","box":[cx,cy,w,h]}, ...]
                                              (의존성 없는 JSON; VLM/다운스트림 연동)

실행:
  python3 nodes/yolo_node.py
  python3 nodes/yolo_node.py --model yolov8n.pt --image-topic /stereo/left/image_raw --conf 0.4
  # 좌/우 카메라도 보려면 노드를 토픽만 바꿔 여러 개 띄우면 됨.

의존:
  pip install ultralytics
  sudo apt install ros-humble-cv-bridge
  (가중치 yolov8n.pt 는 첫 실행 시 자동 다운로드)

주의: 호스트 .bashrc 에 FASTRTPS_DEFAULT_PROFILES_FILE 이 export 돼 있어야
      Isaac 토픽이 보인다 (ros2_bridge_fix 참고).
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

try:
    import json
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloNode(Node):
    def __init__(self, model_path, image_topic, conf):
        super().__init__("yolo_node")
        if YOLO is None:
            self.get_logger().error("ultralytics 미설치 -> pip install ultralytics")
            raise SystemExit(1)
        if CvBridge is None:
            self.get_logger().error("cv_bridge 미설치 -> sudo apt install ros-humble-cv-bridge")
            raise SystemExit(1)

        self.bridge = CvBridge()
        self.conf = conf
        self.get_logger().info(f"YOLO 모델 로드: {model_path}")
        self.model = YOLO(model_path)

        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.img_pub = self.create_publisher(Image, "/yolo/image", 10)
        self.det_pub = self.create_publisher(String, "/yolo/detections_json", 10)
        self.get_logger().info(f"구독 {image_topic} -> /yolo/image, /yolo/detections_json")

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"이미지 변환 실패: {e}")
            return

        result = self.model.predict(frame, conf=self.conf, verbose=False)[0]

        # 주석 이미지 발행
        annotated = result.plot()  # numpy BGR (박스/라벨 그려짐)
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.img_pub.publish(out)

        # 탐지 결과 JSON 발행 + 로깅
        names = self.model.names
        dets = []
        for b in result.boxes:
            cls = int(b.cls[0])
            cx, cy, w, h = (round(v, 1) for v in b.xywh[0].tolist())
            dets.append({
                "name": names.get(cls, str(cls)),
                "conf": round(float(b.conf[0]), 3),
                "box": [cx, cy, w, h],
            })
        self.det_pub.publish(String(data=json.dumps(dets)))
        if dets:
            summary = ", ".join(f"{d['name']}:{d['conf']}" for d in dets)
            self.get_logger().info(f"탐지 {len(dets)}개: {summary}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--image-topic", default="/stereo/left/image_raw")
    ap.add_argument("--conf", type=float, default=0.4)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = YoloNode(args.model, args.image_topic, args.conf)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
