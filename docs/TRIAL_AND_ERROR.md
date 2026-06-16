# Tesla Jr. — 시행착오 전체 기록 (Trial & Error Log)

> Isaac Sim 4.2.0 + ROS2 Humble 환경에서 Carter 검증 → T870 자율주행까지
> 겪은 모든 문제와 해결 과정. 증상(Symptom) → 원인(Cause) → 해결(Fix) 형식.
> 같은 문제 재발 시 빠른 참조용 + 보고서 자료.

목차
- [A. 인프라 / 통신](#a-인프라--통신)
- [B. Isaac Sim Python API 함정](#b-isaac-sim-python-api-함정)
- [C. 센서 / 좌표(TF) / 시간](#c-센서--좌표tf--시간)
- [D. Nav2 / costmap](#d-nav2--costmap)
- [E. T870 모델링 / 임포트](#e-t870-모델링--임포트)
- [F. T870 주행 물리 (가장 고생한 부분)](#f-t870-주행-물리-가장-고생한-부분)
- [G. 핵심 교훈 요약](#g-핵심-교훈-요약)

---

## A. 인프라 / 통신

### A-1. 물리 시뮬이 안 돌고 토픽 데이터 0 (`invalid dt 0.000000`)
- **증상**: Carter 예제 Play 해도 화면 변화 없음. `ros2 topic hz` 무응답.
  콘솔에 `invalid dt 0.000000, skipping current step` 반복.
- **원인**: GPU 격리 문제. `--gpus '"device=0"'` + `--privileged` 조합에서
  Isaac Sim 이 GPU 를 어중간하게 잡아 PhysX 가 GPU 미사용 → dt=0.
  진단: `nvidia-smi` 에서 Isaac GPU VRAM 366MB / util 8%, CPU 372% (비정상).
- **해결**: `run_isaac_gui.sh` 에서 `--gpus all`, `NVIDIA_VISIBLE_DEVICES=all`,
  `--shm-size 16g`. → GPU 5.4GB/100%, 물리 정상.

### A-2. 토픽 이름은 보이는데 데이터가 안 옴 (FastDDS SHM 불일치)
- **증상**: `ros2 topic list` 엔 토픽이 보이나 `echo` 는 영원히 대기.
  `topic info -v` → `Subscription count: 0`. Isaac 내부 publish 는 정상.
- **원인**: 컨테이너 FastDDS 2.6.8 vs 호스트 2.6.11. `--network=host` 라
  공유메모리(SHM) 전송을 쓰는데 버전이 달라 데이터 전달 실패.
  (discovery 는 UDP 멀티캐스트라 동작 → "이름만 보임")
- **해결**: `docker/fastdds.xml` 로 SHM 끄고 UDP-only 강제
  (`useBuiltinTransports=false` + UDPv4 transport). 컨테이너+호스트 양쪽에
  `FASTRTPS_DEFAULT_PROFILES_FILE` 적용. 죽은 `/dev/shm/fastrtps_*` 청소.
  **함정**: 처음 XML 에 `interfaceWhiteList 127.0.0.1` + `initialPeers` 까지
  넣었더니 discovery 까지 막힘. → 멀티캐스트는 살리고 SHM만 꺼야 함.

### A-3. 호스트→Isaac 역방향(/cmd_vel) 전달
- A-2 해결(UDP-only)로 양방향 모두 정상화. discovery 됐다고 데이터가 오는 건
  아니므로 항상 `topic echo` 로 실제 수신 확인.

---

## B. Isaac Sim Python API 함정

### B-1. `rclpy.ok() == False`
- **원인**: Isaac 내장 rclpy 미초기화.
- **해결**: `if not rclpy.ok(): rclpy.init()` 가드. 모든 스크립트에 포함.

### B-2. `ImportError: cannot import name 'XformPrim'`
- **해결**: 4.2.0 은 대문자 F → `from omni.isaac.core.prims import XFormPrim`.

### B-3. `GetPrimAtPath(Stage, str)` Boost.Python.ArgumentError
- **원인**: 이 USD 빌드는 문자열 대신 `Sdf.Path` 요구.
- **해결**: `stage.GetPrimAtPath(Sdf.Path(path))`.

### B-4. `Failed to create simulation view: no active physics scene found`
- **원인**: GUI 모드에서 `world.reset()` 시 PhysicsScene 없음.
- **해결**: `UsdPhysics.Scene.Define` 로 명시 생성 + `world.reset()` try/except.

### B-5. 좀비 물리 콜백 (재실행/씬전환 시 에러 스팸)
- **증상**: `subscribe_physics_step_events` 콜백이 `=None` 으로 해제 안 됨.
  이전 인스턴스가 살아 에러를 뿜음. File>New 후 삭제된 prim 핸들을 계속 읽어
  `DcGetRigidBodyPose: Invalid or expired body handle` 스팸.
- **해결**: `stop()` 에서 `.unsubscribe()` 명시 호출 + `builtins` 전역
  레지스트리로 재실행 시 모든 이전 인스턴스 정리. **씬 전환(File>New) 전엔
  반드시 콜백 스크립트를 먼저 stop()** (콜백은 앱 레벨이라 File>New 로 안 지워짐).

### B-6. `rclpy.spin` 스레드 충돌 (`generator already executing`)
- **원인**: rclpy.spin 을 별도 스레드로 돌리니 다른 spinner 와 충돌.
- **해결**: 전용 `SingleThreadedExecutor` + 물리 콜백에서 `spin_once(timeout_sec=0)`.

### B-7. `World` 싱글톤이 File>New 후에도 살아있음
- **증상**: File>New 후 `add_default_ground_plane()` 가 새 씬에 ground 를 안 만듦
  → 로봇이 바닥을 뚫고 떨어짐.
- **원인**: 이전 세션의 World 싱글톤이 옛 스테이지를 참조.
- **해결**: `World.clear_instance()` 후 재생성 + `GroundPlane` prim 직접 생성.

---

## C. 센서 / 좌표(TF) / 시간

### C-1. 카메라 화면이 까맣다
- **증상**: stereo 이미지 수신은 정상(640x480 rgb8)인데 거의 새까맘(픽셀 mean 1.4).
- **원인**: ROS/QoS 문제가 아니라 **장면이 실제로 어둡고 볼 물체가 없음**.
- **해결**: `setup_scene.py` 로 Dome+Distant 조명 강화 + 전방 색깔 큐브 배치.
- **교훈**: 까만 이미지는 픽셀값부터 확인 (수신 vs 조명 구분).

### C-2. odom 이 로봇 이동을 추적 못 함 (항상 스폰 좌표)
- **증상**: 로봇이 움직여도 /odom 위치가 스폰값 그대로.
- **원인**: 물리 결과는 Fabric(USDRT)에 기록되는데 `UsdGeom.XformCache` 는
  원본 USD 의 정적 authored 값만 읽음.
- **해결**: `omni.isaac.dynamic_control` 로 강체의 실제 pose/속도 직접 읽기.

### C-3. 포인트클라우드가 RViz 에 안 보임 (1) — optical 프레임
- **증상**: 클라우드 수신 정상(307200pts)인데 RViz 표시 안 됨.
- **원인**: 클라우드는 ROS optical 규약(+Z 전방)인데, 발행한 카메라 TF 는
  USD 카메라 prim 프레임(-Z 전방). 클라우드 +Z 가 로봇 뒤로 찍힘.
- **해결**: 카메라 TF 에 **X축 180° 회전**(optical 보정) 적용.

### C-4. 포인트클라우드가 RViz 에 안 보임 (2) — 시간축 불일치
- **증상**: optical 보정 후에도 RViz 가 클라우드를 drop
  (`Message Filter dropping message ... queue is full`).
- **원인**: 클라우드 stamp = Isaac monotonic sim time(예 1542s),
  bridge TF stamp = 직접 누적값(재실행 시 0 리셋, 예 13s). 시간이 안 맞아
  TF 변환 실패.
- **해결**: 전부 **시스템(wall) 시간**으로 통일. 카메라 helper `useSystemTime=True`,
  bridge 는 `node.get_clock().now()`. RViz(기본 wall)와 일치.

---

## D. Nav2 / costmap

### D-1. Nav2 가 로봇을 가둠 (2m 목표인데 경로 14m, 안 움직임)
- **증상**: 목표 (2,0) 인데 distance_remaining 14m, cmd_vel 거의 0.
  local costmap 6400셀 중 2243 점유.
- **원인**: **바닥(ground)이 장애물로 오인식**. 카메라가 본 바닥점이 costmap 에
  들어감. 특히 바닥이 거리에 따라 ~2-3° 떠올라(원거리 z 0.28)
  `min_obstacle_height(0.15)` 를 넘음.
- **해결**: `min_obstacle_height` 0.15 → 0.35. 기운 바닥 제외, 큐브(z≤1.0)는 유지.
  결과: 치명 장애물이 큐브 위치에만 국한.

### D-2. Nav2 DWB 가 Ackermann 과 안 맞음
- **원인**: DWB 는 차동구동용 → 제자리 회전(rotate-in-place) 명령을 냄.
  Ackermann 은 제자리 회전 불가.
- **해결**: 컨트롤러를 **RegulatedPurePursuit(RPP)** 로 교체.
  `use_rotate_to_heading=false`, `yaw_goal_tolerance` 완화(최종 yaw 무시).

---

## E. T870 모델링 / 임포트

### E-1. articulation root 가 `/World/t870` 가 아님
- dynamic_control `get_articulation("/World/t870")` → handle 0 (실패).
- **해결**: ArticulationRootAPI 가 `/World/t870/base_link` 에 있음 →
  `get_articulation("/World/t870/base_link")`.

### E-2. 임포트 직후 로봇이 바닥을 뚫고 떨어짐
- **원인**: B-7 (stale World 싱글톤) 으로 ground 미생성.
- **해결**: `World.clear_instance()` + `GroundPlane` 직접 생성.
- 진단: collider 5개(base+4휠) 정상, ground 섹션 비어있음 → ground 없음 확인.

---

## F. T870 주행 물리 (가장 고생한 부분)

### F-1. 조향(앞바퀴)이 안 돌아감
- **증상**: 조향 조인트 stiffness 1e5 인데 목표각 0.4 줘도 위치 ~0 (안 움직임).
  뒷바퀴(구동)는 정상.
- **원인**: steer 링크 관성이 작은데(0.001) FORCE 드라이브 모드 + 고stiffness 는
  수치적으로 불안정.
- **해결**: 조향을 **ACCELERATION 드라이브 모드**(게인이 관성과 무관)로.
  stiffness 500 / damping 50. (참고: dynamic_control driveMode 는 position/velocity
  가 아니라 FORCE/ACCELERATION 구분. 위치 vs 속도 제어는 stiffness/damping 으로.)

### F-2. 뒷바퀴가 헛돔 (wheelspin / traction 부족) ★최대 난관
- **증상**: 뒷바퀴 5.83 rad/s 정상 회전하는데 로봇이 거의 안 나감.
  base_link z=0.121 (바퀴 기립 정상, 차체 접지 아님).
- **시도 1**: 고마찰 PhysicsMaterial(static 1.5) 적용 → 효과 미미.
- **시도 2**: 앞 rolling 바퀴 damping 0 (브레이크 해제) → 일부 개선.
- **원인(최종)**: URDF 실린더 바퀴가 임포트 시 거친 convex-hull 다면체로 근사
  → 바닥 접촉이 들쭉날쭉(가끔 물고 대부분 미끄러짐).
- **해결**: **바퀴 충돌형상을 sphere(구)로 교체** (URDF `<collision>` cylinder
  → sphere radius 0.12, 시각은 cylinder 유지). 구는 완벽한 점 구름 접촉 →
  안정적 traction. + 클린 컨테이너 재시작.
- **교훈**: sim 바퀴는 sphere 콜라이더가 표준 트릭. 드라이브/물리가 이상하면
  스크립트 반복 재실행보다 **컨테이너 클린 재시작**(누적된 dof 핸들/좀비 정리).

### F-3. 구동 방향 부호 혼동
- 슬립이 심하던 초기엔 측정이 부정확해 방향 판단이 흔들림. traction 해결 후
  명확: 바퀴 축 +Y → omega>0 이 +X 전진. 전진 cmd → Δx +1.29m(+X) 확인.

### F-4. bridge 와 ackermann 의 dynamic_control 충돌 ★
- **증상**: ackermann 단독이면 잘 달리는데, sim_ros2_bridge 까지 켜면
  로봇이 거의 안 움직이고 odom 도 0. (간헐적 `Invalid dof handle` 에러)
- **원인**: 두 스크립트가 각각 dynamic_control 을 호출. bridge 의
  `get_rigid_body` 가 ackermann 의 dof 핸들을 무효화 → 구동 끊김.
- **해결**: **odom/tf/clock 발행을 ackermann 노드에 통합** (하나의 articulation
  핸들 단독 소유). T870 은 sim_ros2_bridge 별도 실행 불필요.
  base pose 는 `get_articulation_root_body` 로 동일 articulation 에서 읽음.

---

## G. 핵심 교훈 요약

1. **데이터로 확인**: "안 된다"를 추측하지 말고 픽셀값/조인트속도/odom/RTF 등
   실측으로 원인을 좁힌다. (까만 화면=조명, 헛돎=traction, 안 움직임=핸들충돌 등)
2. **클린 재시작이 약**: dynamic_control 핸들/좀비 콜백/드라이브 설정이 누적되면
   증상이 꼬인다. 스크립트 반복보다 컨테이너 재시작이 빠를 때가 많다.
3. **하나의 dynamic_control 소유자**: 같은 articulation 을 여러 스크립트가
   건드리면 핸들이 무효화된다. 제어+odom 은 한 노드에서.
4. **시간/좌표 규약**: 토픽/TF/RViz 가 같은 시계(wall)와 프레임 규약(optical)을
   써야 한다. 안 맞으면 데이터는 와도 표시/변환이 안 된다.
5. **sim 바퀴는 sphere 콜라이더**, 조향 같은 저관성 조인트는 ACCELERATION 드라이브.
6. **씬 전환 전 콜백 정리**(stop) — 물리 콜백은 File>New 로 안 지워진다.
