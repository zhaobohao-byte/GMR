# Usage

This file records the common commands for converting BVH or SMPL-X motions to GMR robot motion pkl files.

Run all commands from the repository root:

```bash
cd /home/bobby/gmr/GMR
```

## BVH to GMR

Use the helper script:

```bash
bash scripts/sh_cmd/bvh_gmr.sh
```

Or call the Python entry directly:

```bash
python scripts/bvh_to_robot.py \
  --bvh_file motion_data/selected_motions/straight_sprint_bvh/sprint1_subject4_5978_6058.bvh \
  --format lafan1 \
  --robot ultra \
  --rate_limit \
  --show_skeleton \
  --save_path motion_data/sprint1_subject4_5978_6058.pkl
```

Useful options:

- `--bvh_file`: input BVH file.
- `--format`: BVH parser format, currently `lafan1` or `nokov`.
- `--robot`: target robot. For current ultra workflow, use `ultra`.
- `--save_path`: output GMR robot motion pkl.
- `--rate_limit`: play at motion FPS in the realtime viewer.
- `--show_skeleton`: overlay raw/scaled human skeletons.
- `--headless`: retarget and save without opening the viewer.
- `--loop`: loop playback in the viewer.
- `--record_video --video_path videos/example.mp4`: record viewer video.
- `--motion_fps 30`: set BVH motion FPS.

Headless conversion example:

```bash
python scripts/bvh_to_robot.py \
  --bvh_file motion_data/TPOS.bvh \
  --format lafan1 \
  --robot ultra \
  --headless \
  --save_path motion_data/TPOS_ultra.pkl
```

## SMPL-X to GMR

Use the helper script:

```bash
bash scripts/sh_cmd/smplx_gmr.sh
```

Or call the Python entry directly:

```bash
python scripts/smplx_to_robot.py \
  --smplx_file motion_data/AMASS/ACCAD/Female1Running_c3d/C6_-_stand_to_run_backwards_stageii.npz \
  --robot ultra \
  --rate_limit \
  --show_skeleton \
  --save_path motion_data/smplx_ultra.pkl
```

Useful options:

- `--smplx_file`: input SMPL-X `.npz` or `.pkl` motion file.
- `--robot`: target robot. For current ultra workflow, use `ultra`.
- `--save_path`: output GMR robot motion pkl.
- `--rate_limit`: play at motion FPS in the realtime viewer.
- `--show_skeleton`: overlay raw/scaled SMPL-X skeletons.
- `--show_robot_body_name`: label robot bodies referenced by IK config.
- `--show_human_body_name`: label scaled human target bodies.
- `--hot_reload_ik_config`: reload IK JSON while the viewer is running.
- `--headless`: retarget and save without opening the viewer.
- `--loop`: loop playback in the viewer.
- `--record_video --video_path videos/example.mp4`: record viewer video.

Headless conversion example:

```bash
python scripts/smplx_to_robot.py \
  --smplx_file motion_data/AMASS/ACCAD/Female1Running_c3d/C6_-_stand_to_run_backwards_stageii.npz \
  --robot ultra \
  --headless \
  --save_path motion_data/smplx_ultra.pkl
```

## View a GMR PKL

```bash
python scripts/vis_robot_motion.py \
  --robot ultra \
  --robot_motion_path motion_data/sprint1_subject4_5978_6058.pkl
```

## Plot Ultra Limits

After generating a GMR pkl, plot joint position limits, joint velocity limits, and base x/y/yaw velocity:

```bash
python scripts/plot_robot_motion_limits.py \
  --input_file motion_data/sprint1_subject4_5978_6058.pkl
```

This creates:

- `<input_stem>_joint_pos.png`
- `<input_stem>_joint_vel.png`
- `<input_stem>_base_vel.png`

Plot only selected groups:

```bash
python scripts/plot_robot_motion_limits.py \
  --input_file motion_data/sprint1_subject4_5978_6058.pkl \
  --plot_joints_pos

python scripts/plot_robot_motion_limits.py \
  --input_file motion_data/sprint1_subject4_5978_6058.pkl \
  --plot_joints_vel

python scripts/plot_robot_motion_limits.py \
  --input_file motion_data/sprint1_subject4_5978_6058.pkl \
  --plot_base_vel
```
