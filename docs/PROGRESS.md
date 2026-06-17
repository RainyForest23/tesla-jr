# Tesla Jr. — 진행 상황 & 실행 런북

> 최종 업데이트: 2026-06-16
> 이 문서는 "지금까지 무엇이 되어 있고, 어떻게 다시 실행하는가"를 정리한 런북이다.
> 마주친 버그와 해결 과정은 [`TROUBLESHOOTING.md`](./TROUBLESHOOTING.md) 참고.

---

## 1. 한눈에 보는 현재 상태

```
Isaac Sim 4.2.0 (Docker, GUI)  /World/Carter (nova_carter_sensors)
  │
  ├─ /clock                          시간(wall)                 [rclpy]
  ├─ /odom                           로봇 위치/속도(dynamic_control) [rclpy]
  ├─ /tf                             odom→base_link, base_link→stereo_left [rclpy]
  ├─ /cmd_vel (구독)                 차동 구동 주행             [OmniGraph]
  ├─ /stereo/{left,right}/image_raw  640x480 @~30Hz             [OmniGraph]
  ├─ /stereo/{left,right}/camera_info intrinsics 포함           [OmniGraph]
  ├─ /stereo/left/depth              깊이 이미지(32FC1)         [OmniGraph]
  └─ /stereo/left/points             포인트클라우드 307200pts @~27Hz [OmniGraph]
        │
        ▼  (host ROS2 Humble, FastDDS UDP-only, 전부 wall-clock 시간)
   RViz2  ← stereo 이미지 + odom + TF + 포인트클라우드 시각화
   키보드 텔레옵 ← 방향키/WASD 로 주행

주의: Isaac 카메라/bridge/RViz 모두 시스템(wall) 시간 사용. 카메라 helper는
      useSystemTime=True, bridge 는 node clock. (sim-time 으로 통일하려면 별도 작업)
```

**★ 1차 목표 달성 (2026-06-17): T870 자율주행 + 장애물 회피 시뮬 작동**
카메라→포인트클라우드→costmap→Nav2(Smac Hybrid-A*/RPP)→/cmd_vel→Ackermann→T870 주행→목표 도달.

**Phase 2 — YOLO + JSON Goal 노드 (2026-06-17, 검증 완료)**:
- `nodes/yolo_node.py`: 카메라 → YOLO(yolov8n, GPU ~18Hz) → `/yolo/image` + `/yolo/detections_json`.
  sim에 사람(Isaac People) 배치 → **person 0.82 라이브 탐지 확인**. (`isaac/scripts/add_yolo_target.py`)
- `nodes/json_goal_node.py`: `/goal_json`(JSON) → navigate_to_pose. **VLM→주행목표 다리 검증.**
- 설치: `pip3 install --user ultralytics` + `pip3 install --user "numpy<2"`(필수, cv_bridge 충돌 회피).

**미로 주행 + 가상 범퍼 (2026-06-17)**:
- 단일/이중 L자 코너: footprint+MPPI+persistent costmap 으로 매끈히 완주.
- 4코너 ㄹ자(밀폐 snake) 미로: 첫 코너 진입까지 성공, 완주 실패 → camera-only 한계 확정.
- **가상 범퍼**(`ackermann_control.py`): 후면 카메라 없어 후진 시 뒤 벽 못 봄 →
  "명령했는데 안 움직이면(stuck) 그 방향에 장애물 추론" → `/virtual_bumper/points`
  발행(costmap bump 소스, persistent) + 구동 정지. 바깥 우회 꼼수 차단 검증.
- 결론: 복잡 미로 완주는 지도(SLAM) 필요 = Phase 3. (Tesla식 vision-only 철학 유지)

**코너 주행 + 제어/인식 고도화 (2026-06-17)**: L자 코너(정면벽 회피 + 좌 90° 진입).
- Nav2 footprint 1.0×0.6(원형반경 대신) → 차 길이 인식, 벽 안 붙음.
- 컨트롤러 RPP→**MPPI**(`nav2_mppi_controller`, Ackermann, 후진허용, footprint 충돌검사).
  batch 1000/time_steps 40 경량화(안 그러면 bringup lifecycle 타임아웃).
- **global costmap 누적화(clearing:False)**: 지나친 뒤쪽 벽을 raytrace clearing이
  소거하던 문제 해결 → 본 벽 모두 유지(persistent map). local은 반응형 유지.
- 결과: 2.4m 좁은 코너를 MPPI로 벽에 안 붙고 매끄럽게 완주. `nav2/send_goal.sh x y` 로 목표 전송.

**좌/우 스테레오 카메라 추가 (2026-06-17)**: 실제 T870 설계대로 정면+좌+우 3카메라.
- `isaac/scripts/add_side_cameras.py` (front 불변, 좌/우만 추가).
- 좌/우를 전방 30° 틸트(실제 중심 ±60°) + 화각 ~66°(focal 16mm) → 전방 약 180° 연속 커버.
- 토픽 /left_cam/*, /right_cam/* (image_raw/camera_info/points). 정적 TF
  base_link→{left_cam,right_cam} /tf_static latched (optical 180°X 보정).
- 3 클라우드 모두 Nav2 voxel_layer observation_sources(front/left/right)로 연결.
- 검증: 'gate' 레이아웃(전방-대각선 33~52° 기둥) 목표 (6,0) 통과 — front 화각 밖
  기둥을 좌/우 카메라가 마킹해 회피 성공.

**장애물 회피 검증 (2026-06-17, 클린 재시작 후)**: 목표 (11,0) 자율 도달.
- 장애물①(5,+0.7) → 오른쪽으로 회피 (y −0.13)
- 장애물②(9,−0.7) → 왼쪽으로 회피 (y +0.12)
- ±0.13 완만 S자 (Ackermann 최소반경 1.0m 준수), 목표 10.81에서 정지 (tol 0.35).
- 핵심: NavFn(infeasible 급선회 w~0.48) → Smac Hybrid-A*(완만 feasible) 교체로 해결.
- 주의: recovery 사이클 누적 시 dynamic_control 핸들/물리가 꼬여 전진 명령에도 안 움직임
  → 컨테이너 클린 재시작으로 복구 (재시작 직후 직진 1m 정상 확인).

**달성한 과제 요구사항**
- [x] ROS2 기반 구성
- [x] 센서 데이터 입력 (stereo camera + depth + 포인트클라우드)
- [x] 시각화 (RViz2: 이미지 + odom + TF + 포인트클라우드 + costmap + 경로)
- [x] 주행 (키보드 텔레옵 + Nav2 자율주행)
- [x] 노드 3개 이상 (camera publish / ackermann+odom / Nav2 스택)
- [x] 동작 데모 (T870 자율주행)

## T870 자율주행 실행 순서
1. (호스트) `docker/run_isaac_gui.sh`
2. (Script Editor) `exec(open("/workspace/isaac/scripts/launch_t870.py").read())`
3. **Play(▶)**
4. (Script Editor) `exec(open("/workspace/isaac/scripts/fix_friction.py").read())`
5. (Script Editor) `exec(open("/workspace/isaac/scripts/ackermann_control.py").read())`
   — 구동 + odom/tf/clock 통합 (sim_ros2_bridge 별도 실행 불필요)
6. (호스트 새 터미널) `nav2/run_nav2.sh`
7. 목표: RViz "Nav2 Goal" 또는 `navigate_to_pose` 액션

> depth 는 Isaac ground-truth (1단계). 추후 stereo matching(stereo_image_proc) 교체 예정.
> 남은 고도화: 좌/우 스테레오 추가, YOLO, VLM(Gemma4) goal, T870 실측 치수 반영.

**중요 설계 결정**: 깊이 소스는 **stereo camera**. nova_carter_sensors 모델에 LiDAR도
달려 있으나 프로젝트 설계상 **LiDAR는 사용하지 않는다**.

---

## 2. 핵심 인프라 설정 (이미 반영됨)

### 2.1 Docker 실행 (`docker/run_isaac_gui.sh`)
GUI 모드. 핵심 포인트:
- `--gpus all` / `NVIDIA_VISIBLE_DEVICES=all` — **GPU 격리 문제 해결** (아래 참고)
- `--shm-size=16g`
- `FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/docker/fastdds.xml` — **DDS 데이터 전달 해결**
- ROS2 브리지 환경변수 3종 (RMW / LD_LIBRARY_PATH / AMENT_PREFIX_PATH)

### 2.2 DDS 프로파일 (`docker/fastdds.xml`)
컨테이너(FastDDS 2.6.8)와 호스트(2.6.11)의 버전 차이로 공유메모리(SHM) 전송이
실패 → **SHM 끄고 UDP-only 강제**. 멀티캐스트 discovery는 유지.

### 2.3 호스트 환경변수 (`~/.bashrc`)
```bash
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/Desktop/git_teslaJR/docker/fastdds.xml
```
> 호스트에서 `ros2 ...` 명령을 쓸 때 이 프로파일이 **반드시** 적용돼 있어야 데이터가 보인다.

---

## 3. 처음부터 다시 실행하는 법 (Run Book)

### Step 1 — 컨테이너 시작
```bash
docker stop rainy_isaac 2>/dev/null; docker rm rainy_isaac 2>/dev/null
cd ~/Desktop/git_teslaJR/docker && ./run_isaac_gui.sh
```
Isaac Sim GUI가 뜰 때까지 대기 (`Isaac Sim App is loaded.`).

### Step 2 — 씬 구성 (원클릭 런처, Script Editor)
> Window > Script Editor. **새 씬(File > New)에서 시작 권장.**
```python
exec(open("/workspace/isaac/scripts/launch_all.py").read())
```
spawn_robot + setup_scene + stereo_camera_publish + teleop_cmdvel 를 한 번에 구성.

### Step 3 — Play (▶)
물리 + 모든 OmniGraph 초기화. 로봇이 바닥을 통과해 떨어지면 PhysicsScene 문제 (TROUBLESHOOTING 참고).

### Step 4 — 상태 publish (clock/odom/tf)
> **반드시 Play 이후에 실행** (dynamic_control 핸들이 물리 시작 후 유효).
```python
exec(open("/workspace/isaac/scripts/sim_ros2_bridge.py").read())
```

> 개별 실행이 필요하면 `spawn_robot.py` → Play → 나머지 순으로도 가능.
> OmniGraph 기반(stereo, teleop)은 Play 시점에 초기화되므로, 나중에 추가했다면 Stop→Play 한 번.

### Step 5 — 주행 (선택)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.2}}"
# 멈춤: x=0, z=0 으로 한번 더
```

### Step 6 — 호스트에서 검증
```bash
# 새 터미널 (또는 source ~/.bashrc 이후)
ros2 topic list
ros2 topic hz /stereo/left/image_raw      # ~30Hz 나오면 정상
ros2 topic echo /odom --once
```

### Step 7 — RViz2 시각화
```bash
export DISPLAY=:2
rviz2 -d ~/Desktop/git_teslaJR/isaac/rviz/teslajr.rviz
```
이미지가 검게 보이면 Image 디스플레이의 Reliability Policy를 Best Effort로.

---

## 4. 파일 구조

```
docker/
  run_isaac.sh            headless WebRTC 실행
  run_isaac_gui.sh        X11 GUI 실행 (현재 주력)
  fastdds.xml             DDS UDP-only 프로파일 (SHM 우회)
isaac/
  scripts/
    spawn_robot.py        ground+light+physics+robot 스폰
    sim_ros2_bridge.py    /clock /odom /tf 발행 (rclpy)
    stereo_camera_publish.py  stereo 이미지 발행 (OmniGraph)
  rviz/
    teslajr.rviz          RViz2 시각화 설정
docs/
  PROGRESS.md             (이 문서)
  TROUBLESHOOTING.md      버그/이슈 해결 로그
```

---

## 5. 다음 단계 (Phase 계획)

1. **stereo → depth 노드** (호스트측 첫 처리 노드) — `stereo_image_proc` 또는 커스텀.
   좌/우 → disparity → depth/pointcloud → costmap.
2. **YOLO 객체 탐지 노드** — `/stereo/left/image_raw` 구독 → bbox publish.
3. **Nav2 연동** — costmap + goal → path planning. (odom/tf 이미 준비됨)
4. **VLM(Gemma 4) goal 노드** — JSON 응답 → goal pose 변환.
5. **로봇 주행** — `/cmd_vel` 구독 컨트롤러 추가.
6. **T870 URDF** 임포트 → Ackermann 컨트롤러 전환 → Sim-to-Real.
