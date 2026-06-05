# Ultra GameYaw JSON 配置与反复校验指南

本文说明如何围绕 `assets/ultra_GameYaw/mjcf/ultra_game_yaw.xml` 编写和校验 LAFAN BVH 到 Ultra GameYaw 的 IK JSON 配置，并使用 `motion_data/TPOS.bvh` 做单帧快速评估。

## 目标链路

目标是让这条链路稳定工作：

```text
motion_data/TPOS.bvh
  -> load_bvh_file(..., format="lafan1")
  -> src_human="bvh_lafan1"
  -> general_motion_retargeting/ik_configs/bvh_lafan1_to_ultra.json
  -> assets/ultra_GameYaw/mjcf/ultra_game_yaw.xml
  -> robot qpos
  -> viewer 或 pkl
```

当前 Ultra 已在 `general_motion_retargeting/params.py` 中注册：

```python
ROBOT_XML_DICT["ultra"] = ASSET_ROOT / "ultra_GameYaw" / "mjcf" / "ultra_game_yaw.xml"
IK_CONFIG_DICT["bvh_lafan1"]["ultra"] = IK_CONFIG_ROOT / "bvh_lafan1_to_ultra.json"
ROBOT_BASE_DICT["ultra"] = "base_link"
VIEWER_CAM_DISTANCE_DICT["ultra"] = 2.0
```

运行脚本前确认当前 Python 环境已经安装本项目依赖：

```bash
pip install -e .
```

如果报 `ModuleNotFoundError: No module named 'rich'`、`mujoco` 或 `mink`，先修 Python 环境；这类错误不是 JSON 配置错误。

## Ultra XML 里的可用名称

IK JSON 中的 `robot_root_name` 和 `ik_match_table*` 左侧 key 必须是 MuJoCo XML 里真实存在的 body 名。Ultra XML 当前 body 名如下：

```text
base_link
hip_yaw_l_link
hip_roll_l_link
hip_pitch_l_link
knee_pitch_l_link
ankle_pitch_l_link
hip_yaw_r_link
hip_roll_r_link
hip_pitch_r_link
knee_pitch_r_link
ankle_pitch_r_link
waist_yaw_link
shoulder_pitch_l_link
shoulder_pitch_r_link
```

关节和 actuator 名如下：

```text
hip_yaw_l_joint
hip_roll_l_joint
hip_pitch_l_joint
knee_pitch_l_joint
ankle_pitch_l_joint
hip_yaw_r_joint
hip_roll_r_joint
hip_pitch_r_joint
knee_pitch_r_joint
ankle_pitch_r_joint
waist_yaw_joint
shoulder_pitch_l_joint
shoulder_pitch_r_joint
```

用命令从 XML 重新打印这些名称：

```bash
python - <<'PY'
import xml.etree.ElementTree as ET

root = ET.parse("assets/ultra_GameYaw/mjcf/ultra_game_yaw.xml").getroot()

print("Bodies:")
for elem in root.iter("body"):
    print(elem.attrib.get("name"))

print("\nJoints:")
for elem in root.iter("joint"):
    print(elem.attrib.get("name"))

print("\nActuators:")
for elem in root.iter("position"):
    print(elem.attrib.get("name"), "joint=", elem.attrib.get("joint"))
PY
```

## IK JSON 字段

配置文件路径：

```text
general_motion_retargeting/ik_configs/bvh_lafan1_to_ultra.json
```

建议顶层结构保持为：

```json
{
  "robot_root_name": "base_link",
  "human_root_name": "Hips",
  "ground_height": 0.0,
  "human_height_assumption": 1.8,
  "use_ik_match_table1": true,
  "use_ik_match_table2": true,
  "human_scale_table": {},
  "ik_match_table1": {},
  "ik_match_table2": {}
}
```

字段含义：

- `robot_root_name`：Ultra XML 中的根 body，当前应使用 `base_link`。
- `human_root_name`：LAFAN BVH 的根节点，当前使用 `Hips`。
- `ground_height`：人体目标点的地面高度补偿，先从 `0.0` 开始。
- `human_height_assumption`：JSON 调参时假设的人体身高。`load_bvh_file` 当前返回 `actual_human_height = 1.75`，运行时会用 `actual_human_height / human_height_assumption` 缩放 `human_scale_table`。
- `use_ik_match_table1` / `use_ik_match_table2`：是否启用两轮 IK。
- `human_scale_table`：人体骨段缩放表，优先用它解决人体比例和机器人比例差异。
- `ik_match_table1` / `ik_match_table2`：机器人 body 到人体节点的 IK 约束表。

每条 IK 映射格式：

```json
"robot_body_name": [
  "human_body_name",
  position_weight,
  rotation_weight,
  [x_offset, y_offset, z_offset],
  [qw, qx, qy, qz]
]
```

注意：

- `robot_body_name` 必须来自 Ultra XML 的 body 名。
- `human_body_name` 必须来自 LAFAN BVH loader 输出，例如 `Hips`、`Spine2`、`LeftUpLeg`、`LeftLeg`、`LeftFootMod`、`RightFootMod`。
- 四元数顺序是 `wxyz`，不是 `xyzw`。
- 第一轮 IK 适合放方向约束和主要末端约束；第二轮 IK 适合加强 pelvis、脚、手等关键位置。

## 先做静态校验

每次修改 JSON 后，先跑静态校验，不要直接进 viewer。

```bash
python - <<'PY'
import json
import xml.etree.ElementTree as ET
from pathlib import Path

xml_path = Path("assets/ultra_GameYaw/mjcf/ultra_game_yaw.xml")
cfg_path = Path("general_motion_retargeting/ik_configs/bvh_lafan1_to_ultra.json")

root = ET.parse(xml_path).getroot()
xml_bodies = {elem.attrib["name"] for elem in root.iter("body") if "name" in elem.attrib}

cfg = json.loads(cfg_path.read_text())
used_bodies = {cfg["robot_root_name"]}
used_bodies.update(cfg["ik_match_table1"].keys())
used_bodies.update(cfg["ik_match_table2"].keys())

missing = sorted(used_bodies - xml_bodies)
print("robot_root_name:", cfg["robot_root_name"])
print("missing_body_names:", missing)

if missing:
    raise SystemExit("JSON uses body names that do not exist in Ultra XML")
PY
```

当前仓库里的 `bvh_lafan1_to_ultra.json` 仍包含一组类似 G1 的 body 名，例如：

```text
pelvis
left_hip_yaw_link
left_knee_link
left_ankle_roll_link
torso_link
left_elbow_link
right_wrist_yaw_link
```

这些名称不在 Ultra XML 中。应按 Ultra body 名替换，例如：

```text
pelvis                -> base_link
torso_link            -> waist_yaw_link
left_hip_yaw_link     -> hip_yaw_l_link
left_knee_link        -> knee_pitch_l_link
left_ankle_roll_link  -> ankle_pitch_l_link
right_hip_yaw_link    -> hip_yaw_r_link
right_knee_link       -> knee_pitch_r_link
right_ankle_roll_link -> ankle_pitch_r_link
left_shoulder_yaw_link  -> shoulder_pitch_l_link
right_shoulder_yaw_link -> shoulder_pitch_r_link
```

Ultra 当前 XML 只有肩 pitch body，没有 elbow/wrist body。调参初期建议先删掉或置零 elbow/wrist 条目，等腿、根和肩稳定后再考虑是否扩展机器人模型或改用已有肩部 body 做简化约束。

## TPOS.bvh 单帧校验

`motion_data/TPOS.bvh` 是一帧 BVH，适合快速检查 JSON 是否能完成初始化和第一帧 IK。

确认文件结构：

```bash
rg -n "^(MOTION|Frames:|Frame Time:)" motion_data/TPOS.bvh
wc -l motion_data/TPOS.bvh
```

期望看到：

```text
132:MOTION
133:Frames: 1
134:Frame Time: 0.033333
135 motion_data/TPOS.bvh
```

检查 LAFAN loader 输出了 JSON 中需要的人体节点：

```bash
python - <<'PY'
from general_motion_retargeting.utils.lafan1 import load_bvh_file

frames, height = load_bvh_file("motion_data/TPOS.bvh", format="lafan1")
frame = frames[0]

required = [
    "Hips",
    "Spine2",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFootMod",
    "RightUpLeg",
    "RightLeg",
    "RightFootMod",
    "LeftArm",
    "RightArm",
]

print("frames:", len(frames))
print("height:", height)
print("missing_human_names:", [name for name in required if name not in frame])
PY
```

如果这里缺人体节点，先检查 BVH 是否是 LAFAN 风格；如果缺的是 `LeftFootMod` 或 `RightFootMod`，确认 `format="lafan1"`。

## 单帧 retarget 校验

为了避免打开 viewer 后才发现错误，可以直接初始化 GMR 并 retarget 第一帧：

```bash
python - <<'PY'
import numpy as np
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_bvh_file

frames, height = load_bvh_file("motion_data/TPOS.bvh", format="lafan1")
retargeter = GMR(
    src_human="bvh_lafan1",
    tgt_robot="ultra",
    actual_human_height=height,
    verbose=True,
)

qpos = retargeter.retarget(frames[0])
print("qpos_shape:", qpos.shape)
print("finite:", bool(np.isfinite(qpos).all()))
print("root_pos:", qpos[:3])
print("root_rot_wxyz:", qpos[3:7])
print("dof_pos:", qpos[7:])
PY
```

常见失败：

- `KeyError: 'ultra'`：`params.py` 没有注册 `IK_CONFIG_DICT["bvh_lafan1"]["ultra"]` 或当前环境加载了旧代码。
- `Invalid frame name` / `FrameTask` 相关错误：JSON 左侧 robot body 名不在 XML 中。
- `KeyError: 'LeftFootMod'` 或其它人体节点：JSON 右侧 human body 名不在 BVH loader 输出中。
- MuJoCo mesh 加载失败：先修 XML 中 mesh 路径，路径相对 XML 所在目录解析。

## viewer 与保存回放

当前 `scripts/bvh_to_robot.py` 的 `--robot` 有 choices 限制，列表里还没有 `ultra`。如果要通过该脚本运行，需要先把 `ultra` 加进 choices：

```python
parser.add_argument(
    "--robot",
    choices=[
        "unitree_g1",
        "unitree_g1_with_hands",
        "booster_t1",
        "stanford_toddy",
        "fourier_n1",
        "engineai_pm01",
        "pal_talos",
        "ultra",
    ],
    default="unitree_g1",
)
```

然后用 `TPOS.bvh` 做最小可视化和保存：

```bash
python scripts/bvh_to_robot.py \
  --bvh_file motion_data/TPOS.bvh \
  --format lafan1 \
  --robot ultra \
  --rate_limit \
  --save_path motion_data/ultra_tpos.pkl
```

保存后独立回放：

```bash
python scripts/vis_robot_motion.py \
  --robot ultra \
  --robot_motion_path motion_data/ultra_tpos.pkl
```

如果不想先改 `scripts/bvh_to_robot.py`，可以继续使用上一节的单帧 retarget 脚本做无 viewer 校验。

## 反复调参评估流程

建议按这个顺序循环，每次只改一类参数。

1. 静态校验 body 名。

```bash
python - <<'PY'
import json
import xml.etree.ElementTree as ET
from pathlib import Path

xml_bodies = {
    elem.attrib["name"]
    for elem in ET.parse("assets/ultra_GameYaw/mjcf/ultra_game_yaw.xml").getroot().iter("body")
    if "name" in elem.attrib
}
cfg = json.loads(Path("general_motion_retargeting/ik_configs/bvh_lafan1_to_ultra.json").read_text())
used = {cfg["robot_root_name"], *cfg["ik_match_table1"].keys(), *cfg["ik_match_table2"].keys()}
missing = sorted(used - xml_bodies)
print(missing)
raise SystemExit(1 if missing else 0)
PY
```

2. 单帧 retarget `TPOS.bvh`。

```bash
python - <<'PY'
import numpy as np
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.lafan1 import load_bvh_file

frames, height = load_bvh_file("motion_data/TPOS.bvh", format="lafan1")
retargeter = GMR("bvh_lafan1", "ultra", actual_human_height=height, verbose=False)
qpos = retargeter.retarget(frames[0])
print("qpos_len:", len(qpos))
print("finite:", bool(np.isfinite(qpos).all()))
print("dof_min:", float(np.min(qpos[7:])))
print("dof_max:", float(np.max(qpos[7:])))
PY
```

3. 用 viewer 看 T-Pose。

```bash
python scripts/bvh_to_robot.py \
  --bvh_file motion_data/TPOS.bvh \
  --format lafan1 \
  --robot ultra \
  --rate_limit
```

4. 保存 pkl 并回放，确认保存链路没有把根旋转顺序搞错。

```bash
python scripts/bvh_to_robot.py \
  --bvh_file motion_data/TPOS.bvh \
  --format lafan1 \
  --robot ultra \
  --rate_limit \
  --save_path motion_data/ultra_tpos.pkl

python scripts/vis_robot_motion.py \
  --robot ultra \
  --robot_motion_path motion_data/ultra_tpos.pkl
```

5. 换一条多帧 BVH，再重复同样流程。

```bash
python scripts/bvh_to_robot.py \
  --bvh_file motion_data/selected_motions/run1_subject5.bvh \
  --format lafan1 \
  --robot ultra \
  --rate_limit \
  --save_path motion_data/ultra_run1_subject5.pkl
```

## 调参顺序

优先级建议如下：

1. 先让 JSON 的 robot body 名全部匹配 Ultra XML。
2. 先只保留根、腰、左右腿和左右脚约束，暂时去掉 elbow/wrist 这种 Ultra XML 中不存在的 body。
3. T-Pose 中先看站姿：根高度、左右脚位置、膝盖方向、腰部朝向。
4. 如果整体比例不对，先调 `human_scale_table`，不要先堆 offset。
5. 如果脚踩不到地或穿地，先调 `ground_height` 和脚部 position weight，再调脚部 offset。
6. 如果身体朝向不对，调对应条目的旋转 offset 四元数，确认仍是 `wxyz`。
7. 如果动作抖动，降低冲突 body 的 rotation weight，先保证根和脚稳定。
8. 单帧稳定后再看多帧动作，避免把静态 body 名问题误判成动态调参问题。

## 评估标准

单帧 `TPOS.bvh` 通过标准：

- 静态校验没有 missing body。
- GMR 初始化能打印 Ultra body、DoF、motor 名。
- 第一帧 retarget 返回有限数值，`qpos` 没有 `nan` 或 `inf`。
- viewer 中机器人左右不反，双脚接近地面，躯干朝向可接受。

多帧 BVH 通过标准：

- 能完整跑完并保存 pkl。
- `vis_robot_motion.py` 回放不崩溃。
- 双脚没有明显长期穿地。
- 膝盖方向没有反折。
- 腰部和肩部方向没有持续 90 度或 180 度偏转。

## 批处理注意事项

`scripts/bvh_to_robot_dataset.py` 当前没有 `--robot choices` 限制，可以传 `--robot ultra`，但脚本内部写的是：

```python
retarget = GMR(
    src_human="bvh",
    tgt_robot=args.robot,
    actual_human_height=actual_human_height,
)
```

当前 `params.py` 中没有 `IK_CONFIG_DICT["bvh"]["ultra"]`，因此批处理前需要把这里改为：

```python
retarget = GMR(
    src_human="bvh_lafan1",
    tgt_robot=args.robot,
    actual_human_height=actual_human_height,
)
```

建议先完成单文件和 T-Pose 校验，再修改批处理脚本并跑数据集。
