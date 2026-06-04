# LAFAN 新机器人 GMR 映射接入指南

本文只说明 LAFAN1 BVH 到机器人动作的接入流程。SMPL-X、FBX、Nokov、Xsens 等格式不在本文范围内。

## 目标

新增一个机器人后，需要让 GMR 能完成这条链路：

```text
LAFAN1 BVH -> load_bvh_file(..., format="lafan1") -> src_human="bvh_lafan1" -> IK config -> robot qpos -> 可视化或保存 pkl
```

核心工作有三部分：准备 MuJoCo assets、在 `params.py` 注册机器人、编写并注册 `bvh_lafan1` 的 IK 映射 JSON。

## 1. 准备机器人 assets

1. 在 `assets/` 下新建机器人目录，例如：

```text
assets/my_robot/
```

2. 放入 MuJoCo 可加载的 XML/MJCF 文件和所需 mesh、texture 等资源。建议先单独确认 XML 能被 MuJoCo 加载，避免在 IK 阶段才发现 mesh 路径、joint、actuator 或 body 命名问题。

3. 确认 XML 中有清晰的机器人根 body。这个 body 后面要写入 `ROBOT_BASE_DICT` 和 IK JSON 的 `robot_root_name`。常见根 body 名包括 `pelvis`、`base_link`、`Waist`、`waist_link` 等，具体以新机器人 XML 为准。

4. 确认 XML 中参与 IK 的 body 名。GMR 初始化时会打印 robot DoF、body、motor 名称和 ID，可以先用单条 LAFAN 动作运行一次，根据日志选择要映射的躯干、左右大腿、小腿、脚、上臂、前臂、手等 body。

## 2. 在 `params.py` 注册机器人

编辑 `general_motion_retargeting/params.py`，至少补充以下三处。

1. 在 `ROBOT_XML_DICT` 中加入机器人 ID 到 XML 路径的映射：

```python
ROBOT_XML_DICT = {
    # ... existing robots
    "my_robot": ASSET_ROOT / "my_robot" / "my_robot.xml",
}
```

2. 在 `ROBOT_BASE_DICT` 中加入机器人根 body：

```python
ROBOT_BASE_DICT = {
    # ... existing robots
    "my_robot": "base_link",
}
```

这里的 `base_link` 必须是 XML 中真实存在的 body 名。

3. 在 `VIEWER_CAM_DISTANCE_DICT` 中加入默认相机距离：

```python
VIEWER_CAM_DISTANCE_DICT = {
    # ... existing robots
    "my_robot": 2.0,
}
```

数值只影响可视化相机远近，可以按机器人尺寸调大或调小。

## 3. 创建 LAFAN IK 映射配置

在 `general_motion_retargeting/ik_configs/` 下新增配置文件，例如：

```text
general_motion_retargeting/ik_configs/bvh_lafan1_to_my_robot.json
```

建议从已有 LAFAN 配置复制一份再改，例如：

```text
general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json
general_motion_retargeting/ik_configs/bvh_lafan1_to_n1.json
general_motion_retargeting/ik_configs/bvh_lafan1_to_pm01.json
general_motion_retargeting/ik_configs/bvh_to_talos.json
```

配置文件的顶层字段建议保持如下结构：

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

### LAFAN 人体节点

`load_bvh_file(..., format="lafan1")` 会读取 LAFAN BVH 骨架，并额外生成两个脚部节点：`LeftFootMod` 和 `RightFootMod`。LAFAN 配置中常用的人体节点如下：

- `Hips`：人体根节点。
- `Spine2`：躯干朝向参考。
- `LeftUpLeg`、`RightUpLeg`：左右大腿。
- `LeftLeg`、`RightLeg`：左右小腿。
- `LeftFootMod`、`RightFootMod`：左右脚位置加脚趾朝向，通常比直接用 `LeftFoot`、`RightFoot` 更适合脚底约束。
- `LeftArm`、`RightArm`：左右上臂。
- `LeftForeArm`、`RightForeArm`：左右前臂。
- `LeftHand`、`RightHand`：左右手。

### `human_scale_table`

`human_scale_table` 控制 LAFAN 人体各段在进入 IK 前的缩放。实际运行时还会乘上 `actual_human_height / human_height_assumption`。当前 LAFAN loader 返回的 `actual_human_height` 是固定值 `1.75`。

通常做法是先复制相近机器人配置中的比例，再根据新机器人的身体比例微调：

- 腿太长或脚够不到地面时，优先调 `LeftUpLeg`、`RightUpLeg`、`LeftLeg`、`RightLeg`、`LeftFootMod`、`RightFootMod`。
- 手臂过度伸展或过短时，调 `LeftArm`、`RightArm`、`LeftForeArm`、`RightForeArm`、`LeftHand`、`RightHand`。
- 整体根位置或躯干高度不合适时，调 `Hips`、`Spine2`，必要时再检查 `ground_height`。

### `ik_match_table1` / `ik_match_table2`

两个表都是机器人 body 到人体节点的映射。每个条目的格式是：

```json
"robot_body_name": [
  "human_body_name",
  position_weight,
  rotation_weight,
  [x_offset, y_offset, z_offset],
  [qw, qx, qy, qz]
]
```

含义如下：

- `robot_body_name`：MuJoCo XML 中真实存在的机器人 body 名。
- `human_body_name`：LAFAN 人体节点名，例如 `Hips`、`LeftFootMod`。
- `position_weight`：位置约束权重，越大越强调 xyz 位置跟踪。脚、根节点常用较高位置权重。
- `rotation_weight`：姿态约束权重，越大越强调朝向跟踪。躯干、手臂、脚通常需要姿态约束。
- `[x_offset, y_offset, z_offset]`：加在人体目标点上的位置偏移，用来补偿人体骨架点和机器人 body 原点不一致。
- `[qw, qx, qy, qz]`：旋转偏移，注意是 scalar-first 的 `wxyz` 四元数。

`ik_match_table1` 和 `ik_match_table2` 会按顺序执行两轮 IK。实践上可以把第一轮用于建立整体姿态和主要肢体方向，第二轮用于加强脚、手或根节点等关键末端约束。是否启用由 `use_ik_match_table1`、`use_ik_match_table2` 控制。

## 4. 注册 LAFAN IK 配置

在 `general_motion_retargeting/params.py` 的 `IK_CONFIG_DICT["bvh_lafan1"]` 中加入新机器人：

```python
IK_CONFIG_DICT = {
    "bvh_lafan1": {
        # ... existing robots
        "my_robot": IK_CONFIG_ROOT / "bvh_lafan1_to_my_robot.json",
    },
}
```

`src_human="bvh_lafan1"` 会根据这里找到对应的 JSON。单文件脚本 `scripts/bvh_to_robot.py` 使用的是：

```python
GMR(src_human=f"bvh_{args.format}", tgt_robot=args.robot, ...)
```

因此 `--format lafan1` 会对应到 `IK_CONFIG_DICT["bvh_lafan1"]`。

## 5. 更新脚本中的机器人选项

`scripts/bvh_to_robot.py` 的 `--robot` 使用了 `choices` 限制。如果新增机器人要通过这个单文件脚本运行，需要把机器人 ID 加入 choices：

```python
parser.add_argument(
    "--robot",
    choices=["unitree_g1", "...", "my_robot"],
    default="unitree_g1",
)
```

`scripts/bvh_to_robot_dataset.py` 的 `--robot` 当前没有 choices 限制，传入已在 `params.py` 注册的机器人 ID 即可。

注意：批处理脚本是 LAFAN 批量转换入口，但当前代码中初始化 GMR 时写的是 `src_human="bvh"`。如果本地 `IK_CONFIG_DICT` 没有 `"bvh"` 这个 key，需要把脚本中的 `src_human` 调整为 `"bvh_lafan1"`，或者先使用单文件脚本验证新机器人的 LAFAN 配置。

## 6. 单条动作验证流程

先用一条 LAFAN BVH 验证新映射：

```bash
python scripts/bvh_to_robot.py \
  --bvh_file /path/to/LAFAN1/dance1_subject2.bvh \
  --format lafan1 \
  --robot my_robot \
  --rate_limit \
  --save_path motion_data/lafan1_my_robot/dance1_subject2.pkl
```

建议按这个顺序检查：

1. 程序是否能加载 XML。如果 MuJoCo 报错，先修 assets 路径、mesh、joint、actuator 或 XML 结构。
2. 日志中打印的 robot body 名是否包含 IK JSON 里所有 `robot_body_name`。
3. 是否能完成第一帧 IK。如果报缺少 key，通常是 `IK_CONFIG_DICT` 注册、机器人 ID、body 名或 LAFAN 人体节点名不一致。
4. 可视化中脚是否接近地面、左右是否反了、躯干朝向是否正确。
5. 逐步调整 `human_scale_table`、权重、位置 offset、旋转 offset。

保存出来的 pkl 可以再次独立可视化：

```bash
python scripts/vis_robot_motion.py \
  --robot my_robot \
  --robot_motion_path motion_data/lafan1_my_robot/dance1_subject2.pkl
```

## 7. 批量转换 LAFAN 数据集

单条动作稳定后，再跑批处理：

```bash
python scripts/bvh_to_robot_dataset.py \
  --src_folder /path/to/LAFAN1 \
  --tgt_folder motion_data/LAFAN1_my_robot_gmr \
  --robot my_robot \
  --override
```

输出目录会保留源目录结构，并把 `.bvh` 替换为 `.pkl`。如果不加 `--override`，已经存在的目标 pkl 会被跳过。

## 8. 脚本可选参数说明

### `scripts/bvh_to_robot.py`

用于单个 BVH 文件的 retarget、可视化和可选保存。

- `--bvh_file`：必填，输入 BVH 文件路径。
- `--format`：BVH 格式，LAFAN 使用 `lafan1`。默认是 `lafan1`。
- `--loop`：循环播放动作；不加则播放到最后一帧退出。
- `--robot`：目标机器人 ID，例如 `unitree_g1` 或新增的 `my_robot`。如果新增机器人未加入脚本 choices，会被 argparse 拒绝。
- `--record_video`：录制可视化视频。
- `--video_path`：视频输出路径，默认 `videos/example.mp4`。
- `--rate_limit`：按 `motion_fps` 限速播放，适合观察动作；不加时尽快渲染。
- `--save_path`：保存 retarget 后的机器人动作 pkl；不传则只可视化不保存。
- `--motion_fps`：写入输出 pkl 并用于 viewer 的动作帧率，默认 `30`。LAFAN1 通常使用 30 FPS。

### `scripts/bvh_to_robot_dataset.py`

用于批量处理一个目录下的 LAFAN `.bvh` 文件。

- `--src_folder`：必填，包含 BVH 文件的源目录。脚本会递归遍历子目录。
- `--tgt_folder`：输出目录，默认 `../../motion_data/LAFAN1_g1_gmr`。
- `--robot`：目标机器人 ID，默认 `unitree_g1`。
- `--override`：覆盖已经存在的输出 pkl；不加则跳过已有文件。
- `--target_fps`：默认 `30`。当前脚本解析了该参数，但实际输出中 `fps` 固定使用 LAFAN 的 `30`，没有做重采样。

### `scripts/vis_robot_motion.py`

用于查看已经保存的机器人 pkl。

- `--robot`：目标机器人 ID，默认 `unitree_g1`。
- `--robot_motion_path`：必填，机器人动作 pkl 路径。
- `--record_video`：录制播放视频。
- `--video_path`：视频输出路径，默认 `videos/example.mp4`。

## 9. 常见调参建议

- 如果机器人整体朝向不对，优先检查各条目的旋转 offset，四元数顺序必须是 `wxyz`。
- 如果左右肢体交叉或镜像，检查机器人 body 是否左右写反，也检查 LAFAN 节点是否用了对应的 `Left*` / `Right*`。
- 如果脚滑明显，适当提高 `LeftFootMod`、`RightFootMod` 的位置权重，并检查脚部 body 原点是否接近脚底或脚踝。
- 如果上半身僵硬或抖动，降低手臂末端旋转权重，先保证躯干和腿部稳定，再逐步加强手部约束。
- 如果 IK 不收敛或动作抖动，先减少被强约束的 body 数量，确认根、躯干、左右脚稳定后再加入手臂。
- 如果机器人身高比例和 LAFAN 差异很大，优先调 `human_scale_table`，不要只靠 offset 补偿。
