#!/usr/bin/env bash
cd "$(dirname "$0")/../.."
# python scripts/bvh_to_robot.py --bvh_file motion_data/TPOS.bvh --format lafan1 --robot ultra --rate_limit --loop "$@"
python scripts/bvh_to_robot.py --bvh_file motion_data/selected_motions/run1_subject5.bvh --format lafan1 --robot ultra --rate_limit --loop "$@"



# 命令行参数：
# --save_path <path_to_save_robot_data.pkl> # 如果需要保存pkl
# --loop # 如果需要循环播放
# --record_video # 如果需要录制视频
# --video_path <path_to_save_video.mp4> # 保存视频
# --rate_limit # 
# --motion_fps <fps> # 设置动作帧率
# --robot <path_to_robot_data> # 设置机器人
# --format <format> # 设置bvh格式
# --bvh_file <path_to_bvh_data> # 设置bvh文件
# --smplx_file <path_to_smplx_data> # 设置smplx文件