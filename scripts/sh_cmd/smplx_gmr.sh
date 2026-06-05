#!/usr/bin/env bash
cd "$(dirname "$0")/../.."

python scripts/smplx_to_robot.py \
  --smplx_file "motion_data/AMASS/ACCAD/Female1Running_c3d/C5_-_walk_to_run_stageii.npz" \
  --robot "unitree_g1" \
  --rate_limit \
  --loop \
  --show_skeleton \
  "$@"

# 常用命令行参数：
# SMPLX_FILE=<path_to_smplx_data.npz> # 设置 SMPLX 文件
# ROBOT=<robot_name> # 设置机器人，默认 ultra
# --save_path <path_to_save_robot_data.pkl> # 如果需要保存 pkl
# --loop # 如果需要循环播放
# --record_video # 如果需要录制视频，视频路径自动生成到 videos/
# --rate_limit # 按动作帧率限速播放
