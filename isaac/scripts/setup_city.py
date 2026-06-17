"""
Tesla Jr. - 도시 데모 씬: 도로 + 횡단보도 + 건너는 사람
=================================================================
차(원점,+X)가 도로를 따라 주행 -> 전방 횡단보도(x=CROSSWALK_X)를 보행자가 왕복.
YOLO 가 'person' 으로 탐지, 이후 VLM 이 "보행자 -> 정지" 판단하는 데모용.

구성:
  - 도로/횡단보도: 시각 전용 박스(충돌 없음). 차는 기존 GroundPlane 위를 주행.
  - 사람: Isaac People 캐릭터를 물리 콜백으로 횡단보도(Y축) 왕복 이동(애니메이션).

사용법 (Script Editor, Play 중이어도 됨 / 권장 Play 후):
  exec(open("/workspace/isaac/scripts/setup_city.py").read())
중지(사람 이동만): walker.stop()
"""
import builtins
import math
import numpy as np
import omni.usd
import omni.physx
import omni.client
from pxr import UsdGeom, UsdLux, Gf, Sdf
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim

ROAD_LEN = 28.0          # 도로 길이(X)
ROAD_WIDTH = 6.0         # 도로 폭(Y)
CROSSWALK_X = 6.0        # 횡단보도 위치(X)
WALK_SPAN = 2.5          # 보행자 왕복 반경(Y: -SPAN..+SPAN)
WALK_SPEED = 0.8         # 보행 속도(m/s)
CITY_ROOT = "/World/city"


def _box(stage, path, pos, scale, color):
    """시각 전용 박스(충돌 없음). pos/scale=(x,y,z), color=(r,g,b)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*scale))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube


def _find_character(root):
    base = root + "/Isaac/People/Characters"
    try:
        res, entries = omni.client.list(base)
    except Exception:
        return None
    for f in [e.relative_path for e in entries
              if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN]:
        cand = f"{base}/{f}/{f}.usd"
        r2, _ = omni.client.stat(cand)
        if r2 == omni.client.Result.OK:
            return cand
    return None


def build_city():
    stage = omni.usd.get_context().get_stage()
    # 기존 장애물/타겟/도시 제거
    world = stage.GetPrimAtPath("/World")
    for c in list(world.GetChildren()):
        n = c.GetName()
        if n.startswith("obstacle_") or n in ("yolo_target", "city"):
            stage.RemovePrim(c.GetPath())

    # 도로 (어두운 아스팔트), 중앙 차선(노랑), 횡단보도(흰 줄무늬)
    cx = ROAD_LEN / 2.0 - 2.0
    _box(stage, f"{CITY_ROOT}/road", (cx, 0.0, 0.005),
         (ROAD_LEN, ROAD_WIDTH, 0.01), (0.18, 0.18, 0.2))
    # 중앙선 (점선 대신 단순 실선)
    _box(stage, f"{CITY_ROOT}/centerline", (cx, 0.0, 0.012),
         (ROAD_LEN, 0.15, 0.005), (0.9, 0.8, 0.1))
    # 횡단보도 줄무늬 (X로 나열, 각 줄은 Y로 길게)
    for i in range(7):
        sx = CROSSWALK_X - 0.9 + i * 0.3
        _box(stage, f"{CITY_ROOT}/zebra_{i}", (sx, 0.0, 0.013),
             (0.18, ROAD_WIDTH - 1.0, 0.004), (0.92, 0.92, 0.92))
    print(f"도로/횡단보도 생성 (도로 {ROAD_LEN}x{ROAD_WIDTH}, 횡단보도 x={CROSSWALK_X})")

    # 보행자 캐릭터
    root = get_assets_root_path()
    usd = _find_character(root) if root else None
    if usd:
        add_reference_to_stage(usd_path=usd, prim_path=f"{CITY_ROOT}/pedestrian")
        XFormPrim(f"{CITY_ROOT}/pedestrian").set_world_pose(
            position=np.array([CROSSWALK_X, -WALK_SPAN, 0.0]),
            orientation=np.array([0.707, 0.0, 0.0, 0.707]),  # +Y 향함
        )
        print(f"보행자 배치: {usd.split('/')[-1]} @ 횡단보도")
        return True
    print("!!! 보행자 에셋 못 찾음 (도로/횡단보도만 생성됨)")
    return False


class Walker:
    """보행자를 횡단보도(Y축)로 왕복 이동시키는 물리 콜백 애니메이터."""
    def __init__(self, prim_path):
        self.prim = XFormPrim(prim_path)
        self.t = 0.0
        self._sub = (omni.physx.get_physx_interface()
                     .subscribe_physics_step_events(self._step))
        print(">>> 보행자 보행 시작 (중지: walker.stop())")

    def _step(self, dt):
        self.t += dt
        period = 2.0 * WALK_SPAN / WALK_SPEED
        phase = (self.t % (2 * period)) / period   # 0..2
        if phase <= 1.0:
            y = -WALK_SPAN + 2 * WALK_SPAN * phase   # -SPAN -> +SPAN
            q = np.array([0.707, 0.0, 0.0, 0.707])   # +Y
        else:
            y = WALK_SPAN - 2 * WALK_SPAN * (phase - 1.0)  # +SPAN -> -SPAN
            q = np.array([0.707, 0.0, 0.0, -0.707])  # -Y
        try:
            self.prim.set_world_pose(
                position=np.array([CROSSWALK_X, y, 0.0]), orientation=q)
        except Exception:
            pass

    def stop(self):
        if self._sub is not None:
            try:
                self._sub.unsubscribe()
            except Exception:
                pass
        self._sub = None
        print(">>> 보행자 보행 중지")


# ---- 실행 ----
_old = getattr(builtins, "_TESLA_WALKER", None)
if _old is not None:
    try:
        _old.stop()
    except Exception:
        pass

# 조명 보장 (도시 씬이 밝도록)
_st = omni.usd.get_context().get_stage()
_dome = UsdLux.DomeLight.Get(_st, "/World/DomeLight")
if not _dome:
    _dome = UsdLux.DomeLight.Define(_st, "/World/DomeLight")
_dome.CreateIntensityAttr(1200.0)

if build_city():
    walker = Walker(f"{CITY_ROOT}/pedestrian")
    builtins._TESLA_WALKER = walker
print(">>> 도시 씬 준비 완료. Play 중이면 보행자가 횡단보도를 왕복함.")
print(">>> 차 주행: send_goal.sh 로 +X 목표(예: 12 0) 주면 횡단보도로 접근.")
