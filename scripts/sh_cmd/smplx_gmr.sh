#!/usr/bin/env bash
cd "$(dirname "$0")/../.."

# SMPLX_FILE="${SMPLX_FILE:-motion_data/AMASS/ACCAD/Female1Running_c3d/C5_-_walk_to_run_stageii.npz}"
# ROBOT="${ROBOT:-ultra}"

python scripts/smplx_to_robot.py \
  --smplx_file /home/bobby/gmr/GMR/motion_data/AMASS/ACCAD/Female1Running_c3d/C6_-_stand_to_run_backwards_stageii.npz \
  --robot "ultra" \
  --rate_limit \
  --loop \
  --show_skeleton \

  "$@"
  # --smplx_file ""/home/bobby/gmr/GMR/motion_data/AMASS/ACCAD/Female1General_c3d/A1_-_Stand_stageii.npz \

# 参数填写：
# SMPLX_FILE=<path_to_smplx_data.npz> # 设置 SMPLX 文件，默认使用上面的 AMASS 示例
# ROBOT=<robot_name>                  # 设置机器人，默认 ultra
#
# 脚本默认已带：
# --rate_limit # 按动作帧率限速播放
# --loop       # 循环播放
#
# 可追加传给 scripts/smplx_to_robot.py 的参数：
# --smplx_file <path_to_smplx_data.npz> # 也可直接覆盖 SMPLX 文件
# --robot <robot_name>                  # 也可直接覆盖机器人
# --save_path <path_to_save_robot_data.pkl> # 保存 retarget 后的 pkl
# --record_video                        # 录制视频，未指定 video_path 时自动保存到 videos/
# --video_path <path_to_save_video.mp4> # 指定视频保存路径
# --headless                            # 不打开实时 viewer，只做 retarget / 保存
# --show_skeleton                       # 显示原始和缩放后的 SMPL-X 骨架
# --show_robot_body_name                # 显示 IK JSON 中机器人 body 名
# --show_human_body_name                # 显示人体目标 body 名
# --hot_reload_ik_config                # viewer 运行时热加载 IK JSON
#
# 示例：
# bash scripts/sh_cmd/smplx_gmr.sh --record_video
# SMPLX_FILE=motion_data/xxx.npz ROBOT=ultra bash scripts/sh_cmd/smplx_gmr.sh --show_skeleton
