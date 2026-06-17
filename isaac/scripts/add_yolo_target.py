"""
Tesla Jr. - YOLO 탐지 데모용 객체(사람) 배치
=================================================================
로봇 전방에 Isaac '사람' 캐릭터를 세워 YOLO 가 'person' 으로 탐지하는 걸
보여준다. 회색 큐브는 COCO 클래스가 아니라 안 잡히므로 실제 객체 필요.

동작: 기존 장애물(미로 등) 제거 -> 전방 (X_AHEAD, 0) 에 사람 배치(로봇 향함).
에셋 경로는 컨테이너마다 다르므로 People/Characters 를 자동 조회해 첫 캐릭터 사용.

사용법 (Script Editor):
  exec(open("/workspace/isaac/scripts/add_yolo_target.py").read())
  # -> Play(▶)  (카메라 렌더 시작 -> 호스트 yolo_node 가 person 탐지)
"""
import omni.usd
import omni.client
from pxr import UsdGeom, Gf, Sdf
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

X_AHEAD = 4.0          # 로봇(원점,+X) 전방 거리
TARGET_PRIM = "/World/yolo_target"


def _clear_old(stage):
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return
    for c in world.GetChildren():
        n = c.GetName()
        if n.startswith("obstacle_") or n == "yolo_target":
            stage.RemovePrim(c.GetPath())


def _find_character(root):
    """People/Characters 폴더에서 첫 캐릭터 .usd 경로 반환."""
    base = root + "/Isaac/People/Characters"
    try:
        res, entries = omni.client.list(base)
    except Exception as e:
        print(f"Characters 목록 조회 실패: {e}")
        return None
    folders = [e.relative_path for e in entries
               if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN]
    print(f"사용 가능 캐릭터: {folders[:8]}")
    for f in folders:
        cand = f"{base}/{f}/{f}.usd"
        r2, _ = omni.client.stat(cand)
        if r2 == omni.client.Result.OK:
            return cand
    # 폴더명과 파일명이 다른 경우: 폴더 안 첫 .usd
    for f in folders:
        r3, sub = omni.client.list(f"{base}/{f}")
        if r3 == omni.client.Result.OK:
            for s in sub:
                if s.relative_path.endswith(".usd"):
                    return f"{base}/{f}/{s.relative_path}"
    return None


stage = omni.usd.get_context().get_stage()
_clear_old(stage)

root = get_assets_root_path()
print(f"assets root: {root}")
usd = _find_character(root) if root else None

if usd:
    print(f"사람 에셋: {usd}")
    add_reference_to_stage(usd_path=usd, prim_path=TARGET_PRIM)
    # 정밀도 충돌 피하려 XFormPrim 으로 위치/방향 설정 (캐릭터는 Z-up 기립 기본).
    # orientation wxyz=[0,0,0,1] = yaw 180° -> 로봇(원점) 쪽(-X) 바라봄.
    import numpy as np
    from omni.isaac.core.prims import XFormPrim
    XFormPrim(TARGET_PRIM).set_world_pose(
        position=np.array([X_AHEAD, 0.0, 0.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    print(f">>> 사람 배치 완료 @({X_AHEAD},0). Play -> YOLO 가 'person' 탐지할 것.")
else:
    print("!!! 사람 에셋 못 찾음. assets root/People 경로 확인 필요.")
    print(">>> 위 'assets root' 출력을 알려주면 경로를 맞춰줄게.")
