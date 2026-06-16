# Tesla Jr. — 트러블슈팅 로그

> 이 문서는 Isaac Sim ↔ ROS2 연동 과정에서 마주친 버그와 해결 과정을 기록한다.
> 같은 증상이 재발했을 때 빠르게 원인을 찾기 위한 참조용.

---

## 가장 중요한 두 가지 (이번 세션의 핵심)

### ★ 이슈 A — 물리 시뮬레이션이 안 돌고 토픽 데이터가 안 나옴 (GPU 격리)

**증상**
- Carter Navigation 예제 Play → 화면 변화 없음
- `ros2 topic list`엔 토픽이 보이는데, 콘솔에 아래 경고 반복:
  ```
  [Warning] [omni.graph.core.plugin] .../differential_controller_01:
  invalid dt 0.000000, cannot check for acceleration limits, skipping current step
  ```
- `ros2 topic hz /...` 무응답 (데이터 0)

**진단**
`nvidia-smi` + `docker stats` 결과:
| 항목 | 비정상 값 | 정상 기대값 |
|------|-----------|-------------|
| Isaac GPU VRAM | 366MB | 4~6GB |
| GPU Util | 8% | 60~100% |
| CPU | 333% | 100~200% |

GPU가 거의 노는데 CPU만 과부하 → 물리(PhysX)가 GPU로 안 돌고 dt=0으로 스텝이
진행되지 않음. 원인은 `run_isaac_gui.sh`의 GPU 옵션:
- `--gpus '"device=0"'` + `--privileged` 조합에서 GPU 격리가 어긋남
- Isaac Sim이 GPU 0/1을 어중간하게 잡아 PhysX 초기화 실패

**해결** (`docker/run_isaac_gui.sh`)
```diff
- --gpus '"device=0"'
+ --gpus all
- -e NVIDIA_VISIBLE_DEVICES=0
+ -e NVIDIA_VISIBLE_DEVICES=all
- --shm-size=8g
+ --shm-size=16g
```
재시작 후: GPU 0 = 5.4GB/100%, GPU 1 = 4.1GB/78%. 물리 정상 작동, dt 경고 사라짐.

---

### ★ 이슈 B — 토픽 이름은 보이는데 데이터가 안 옴 (FastDDS SHM 불일치)

**증상**
- `ros2 topic list` → 토픽 이름은 보임 (discovery OK)
- `ros2 topic echo` → 영원히 대기, 데이터 0
- `ros2 topic info -v` → `Subscription count: 0`, Node name `UNKNOWN`
- Isaac Sim 내부 Script Editor에서 rclpy 퍼블리셔는 `Published: Hello N` 정상 출력
  → 즉 **발행은 되는데 호스트가 수신 못 함** (data plane 실패)

**진단**
- 컨테이너 FastDDS = `libfastrtps.so.2.6.8`, 호스트 = `2.6.11` (버전 불일치)
- `--network=host`라 FastDDS가 **공유메모리(SHM) 전송**을 선호하는데,
  버전이 다른 두 FastDDS가 SHM 세그먼트를 공유 못 해 데이터 전달 실패
- discovery는 UDP 멀티캐스트라 동작 → 그래서 "이름은 보이고 데이터는 안 옴"
- `/dev/shm`에 죽은 `fastrtps_*` 세그먼트 39개 (다른 유저 잔재 포함)

**해결** (`docker/fastdds.xml` + 양쪽 적용)
SHM을 끄고 UDP-only로 강제하는 프로파일 작성:
```xml
<profiles ...>
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>CustomUdpTransport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only_profile" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>CustomUdpTransport</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>   <!-- SHM 끔 -->
    </rtps>
  </participant>
</profiles>
```
- 컨테이너: `run_isaac_gui.sh`에 `FASTRTPS_DEFAULT_PROFILES_FILE` 환경변수 추가
- 호스트: `~/.bashrc`에 동일 export 추가
- 죽은 SHM 청소: `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`

결과: 호스트에서 `/clock /odom /tf`, stereo 이미지 ~30Hz 정상 수신.

> **함정**: 처음 만든 XML이 `interfaceWhiteList 127.0.0.1` + `initialPeersList`까지
> 넣어 너무 공격적이라 **discovery까지 막혔다**. → 멀티캐스트 discovery는 살리고
> SHM만 끄는 최소 설정이 정답.

---

## 그 외 이슈 (시간순)

### 1. `UsdContext busy` (Play 시점)
씬이 완전히 로드되기 전에 Play를 눌러 Action Graph 초기화 실패.
→ 로딩 progress bar가 사라질 때까지 기다린 후 Play. 필요시 File > New로 재로드.

### 2. `omni.isaac.ros2_bridge ... shutdown` 후 Isaac Sim 크래시
ros2_bridge 확장이 스스로 종료되며 앱 멈춤.
→ 컨테이너 재시작. (근본 원인은 이슈 A의 GPU 문제로 추정)

### 3. `rclpy.ok() == False` (Isaac Sim 내부)
Isaac Sim 내장 rclpy가 초기화되지 않은 상태.
→ Script Editor에서 `import rclpy; rclpy.init()` 먼저 실행.
→ 스크립트들은 `if not rclpy.ok(): rclpy.init()` 가드 포함.

### 4. `ImportError: cannot import name 'XformPrim'`
Isaac Sim 4.2.0에서 클래스명이 대문자 F.
```diff
- from omni.isaac.core.prims import XformPrim
+ from omni.isaac.core.prims import XFormPrim
```

### 5. `Failed to create simulation view: no active physics scene found`
GUI 모드에서 `world.reset()` 호출 시 PhysicsScene이 없어 실패.
→ `spawn_robot.py`에서 `UsdPhysics.Scene`을 명시적으로 생성하고,
   `world.reset()`은 try/except로 감쌈 (Play가 어차피 물리를 초기화).

### 6. `GetPrimAtPath(Stage, str)` Boost.Python.ArgumentError
이 USD 빌드는 `GetPrimAtPath`에 문자열이 아닌 `Sdf.Path`를 요구.
```diff
- self._stage.GetPrimAtPath(ROBOT_PRIM_PATH)
+ self._stage.GetPrimAtPath(Sdf.Path(ROBOT_PRIM_PATH))
```

### 7. 좀비 물리 콜백 (에러 스팸)
`subscribe_physics_step_events`로 등록한 콜백이 `self._phys_sub = None`만으로는
구독 해제가 안 됨 → 이전 exec의 bridge 인스턴스들이 계속 살아 에러를 뿜음.
→ `stop()`에서 `.unsubscribe()` 명시 호출 + `builtins`에 전역 레지스트리를 두어
   재실행 시 모든 이전 인스턴스를 정리. (이미 떠 있는 좀비는 앱 재시작으로만 청소)

### 8. `OgnROS2CameraHelper: camera_info is deprecated` (경고만)
Isaac Sim 4.1.0부터 camera_info는 `OgnROS2CameraInfoHelper` 권장.
→ 현재 동작에는 문제 없음 (경고만). 추후 정리 가능.

---

## 빠른 점검 체크리스트 (데이터 안 보일 때)

1. Isaac Sim에서 **Play(▶) 눌렀나?** (Pause면 발행 안 됨)
2. `nvidia-smi` — Isaac GPU util이 충분히 높은가? (낮으면 이슈 A)
3. 콘솔에 `invalid dt 0.000000` 있나? (있으면 물리 미작동)
4. 호스트 터미널에 `FASTRTPS_DEFAULT_PROFILES_FILE` 적용됐나? (`echo $FASTRTPS_DEFAULT_PROFILES_FILE`)
5. Script Editor에서 `rclpy.ok()` True인가?
6. `ros2 topic info <topic> -v` — Subscription count, Node name이 정상인가?
