# Tesla Jr. — 호스트 ROS2 노드 (Phase 2)

Isaac Sim 밖, **호스트(Ubuntu, ROS2 Humble)** 에서 도는 standalone rclpy 노드들.
colcon 패키지 없이 `python3 nodes/<파일>` 로 바로 실행.

> 모든 노드는 호스트 `.bashrc` 에 `FASTRTPS_DEFAULT_PROFILES_FILE` 가 export 돼
> 있어야 Isaac/Nav2 토픽이 보인다 (UDP-only DDS 프로파일).

---

## 1. `yolo_node.py` — YOLO 객체탐지

카메라 이미지 구독 → YOLO 추론 → 주석 이미지 + 탐지 JSON 발행.

**의존성 설치 (최초 1회)**
```bash
pip3 install --user ultralytics      # torch(CUDA) 포함
pip3 install --user "numpy<2"        # ★ 필수: ultralytics가 numpy2를 끌어오는데
                                     #   ROS2 cv_bridge/matplotlib는 numpy1.x 컴파일 ->
                                     #   안 맞추면 '_ARRAY_API not found' 로 import 실패
sudo apt install ros-humble-cv-bridge
```
> rclpy 가 시스템 파이썬이라 ultralytics 도 같은 파이썬(--user)에 설치해야 함.
> 검증: GPU 추론 ~18Hz, /yolo/image 발행 확인됨 (2026-06-17).

**실행**
```bash
python3 nodes/yolo_node.py                       # 기본: /stereo/left/image_raw
python3 nodes/yolo_node.py --image-topic /left_cam/image_raw --conf 0.5
```

**토픽**
- 구독: `/stereo/left/image_raw` (sensor_msgs/Image)
- 발행: `/yolo/image` (박스 그린 이미지 — RViz Image 디스플레이로 확인)
- 발행: `/yolo/detections_json` (std_msgs/String, `[{"name","conf","box":[cx,cy,w,h]}]`)

좌/우 카메라까지 보려면 `--image-topic` 만 바꿔 여러 개 띄우면 된다.

---

## 2. `json_goal_node.py` — JSON Goal → Nav2

JSON 목표를 받아 Nav2 `navigate_to_pose` 액션으로 전송. **VLM(Gemma) 출력 →
주행 목표** 변환 다리. (`nav2/send_goal.sh` 의 프로그래밍/JSON 버전)

**실행** (Nav2 가 떠 있어야 함)
```bash
python3 nodes/json_goal_node.py
# 다른 터미널에서 목표 전송:
ros2 topic pub --once /goal_json std_msgs/msg/String "{data: '{\"x\":2.5,\"y\":4.0}'}"
ros2 topic pub --once /goal_json std_msgs/msg/String "{data: '{\"x\":6.0,\"y\":0.0,\"yaw\":90}'}"
```

**토픽/액션**
- 구독: `/goal_json` (std_msgs/String — JSON `{"x","y","yaw"(deg, 선택)}`)
- 호출: `navigate_to_pose` (nav2_msgs/action) — 수락/완료를 콘솔에 로깅

---

## 다음 (Phase 2+)
- VLM(Gemma 4) 노드: 장면/이미지 → JSON 목표 생성 → `/goal_json` 발행 (이 노드가 받아 주행).
- `/yolo/detections_json` → 의미 기반 목표(예: "사람 앞에서 정지") 로 연결.
