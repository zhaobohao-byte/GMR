#!/usr/bin/env bash
cd "$(dirname "$0")/../.."

python scripts/gvhmr_to_robot.py \
  --gvhmr_pred_file /home/bobby/gmr/GVHMR/outputs/demo/tennis/hmr4d_results.pt \
  --robot unitree_g1 \
  --rate_limit \
  --loop \
  "$@"

# 常用命令行参数：
# --save_path <path_to_save_robot_data.pkl> # 如果需要保存 pkl
# --loop # 如果需要循环播放
# --record_video # 如果需要录制视频，视频路径由 gvhmr_to_robot.py 自动生成到 videos/
# --rate_limit # 按动作帧率限速播放
# --robot <robot_name> # 设置机器人，也可以用 ROBOT 环境变量
# --gvhmr_pred_file <path_to_hmr4d_results.pt> # 设置 GVHMR 预测文件，也可以用 GVHMR_PRED_FILE 环境变量
