# Tesla Jr. - Vision-Driven Autonomous Navigation System

> "See Local. Think Local. Drive Real."
> 쌍안(stereo) 카메라 기반 깊이 추출 + 로컬 VLM(Gemma 4)로 실시간 자율주행을 수행하는
> 분산 엣지 시스템. Isaac Sim 시뮬레이션 → 실차(Henes Broon T870) 이식.

## 환경
- Ubuntu 22.04 Server (Alienware Aurora R12, RTX 3060 ×2)
- Isaac Sim 4.2.0 (Docker)
- ROS2 Humble + Nav2

## 현재 상태 (2026-06-16)
Isaac Sim → 호스트 ROS2 연동 완료. `/clock` `/odom` `/tf` + stereo 카메라
(`/stereo/{left,right}/image_raw` @~30Hz) 발행 및 RViz2 시각화 동작.

자세한 진행 상황·실행법은 **[`docs/PROGRESS.md`](docs/PROGRESS.md)**,
버그/해결 기록은 **[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md)** 참고.

## 빠른 실행
```bash
# 1) 컨테이너 시작
~/Desktop/git_teslaJR/docker/run_isaac_gui.sh

# 2) Isaac Sim Script Editor 에서 순서대로
exec(open("/workspace/isaac/scripts/spawn_robot.py").read())     # 로봇 스폰
# -> Play(▶)
exec(open("/workspace/isaac/scripts/sim_ros2_bridge.py").read()) # clock/odom/tf
exec(open("/workspace/isaac/scripts/stereo_camera_publish.py").read()) # stereo

# 3) 호스트에서 시각화
rviz2 -d ~/Desktop/git_teslaJR/isaac/rviz/teslajr.rviz
```

## 디렉토리
```
docker/   Isaac Sim Docker 실행 스크립트 + DDS 프로파일
isaac/    Isaac Sim 측 스크립트(scripts) + RViz 설정(rviz)
docs/     진행 상황 런북 + 트러블슈팅 로그
```
