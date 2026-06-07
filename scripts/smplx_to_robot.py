import argparse
import pathlib
import os
import time

import numpy as np
from smplx.joint_names import JOINT_NAMES
from tqdm import tqdm

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.robot_motion_viewer import get_gmr_robot_body_names
from general_motion_retargeting.utils.smpl import load_smplx_file, get_smplx_data_offline_fast

from rich import print


def make_smplx_skeleton_metadata(body_model):
    joint_names = JOINT_NAMES[: len(body_model.parents)]
    return {
        "joint_names": list(joint_names),
        "parents": np.asarray(body_model.parents, dtype=int),
    }


def make_raw_skeleton_pos_offset(raw_frame, scaled_frame, metadata):
    root_name = metadata["joint_names"][0]
    if root_name not in raw_frame or root_name not in scaled_frame:
        return np.zeros(3)
    return np.asarray(scaled_frame[root_name][0]) - np.asarray(raw_frame[root_name][0])


def make_human_skeleton_payloads(raw_frame, scaled_frame, metadata):
    raw_pos_offset = make_raw_skeleton_pos_offset(raw_frame, scaled_frame, metadata)
    return [
        {
            "motion_data": raw_frame,
            "joint_names": metadata["joint_names"],
            "parents": metadata["parents"],
            "rgba": [1.0, 0.45, 0.05, 0.45],
            "joint_radius": 0.008,
            "bone_width": 0.004,
            "pos_offset": raw_pos_offset,
            "show_frames": True,
            "frame_scale": 0.03,
            "frame_arrow_width": 0.0015,
        },
        {
            "motion_data": scaled_frame,
            "joint_names": metadata["joint_names"],
            "parents": metadata["parents"],
            "rgba": [0.05, 0.75, 1.0, 0.75],
            "joint_radius": 0.009,
            "bone_width": 0.005,
            "connect_to_nearest_available": True,
        },
    ]


def make_robot_frame_overlay_options(show_skeleton=False, show_robot_body_name=False):
    return {
        "show_robot_body_name": show_robot_body_name,
        "robot_frame_scale": 0.06 if show_skeleton else 0.03,
    }


if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--smplx_file",
        help="SMPLX motion file to load.",
        type=str,
        # required=True,
        default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male1General_c3d/General_A1_-_Stand_stageii.npz",
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male2MartialArtsKicks_c3d/G8_-__roundhouse_left_stageii.npz"
        # default="/home/yanjieze/projects/g1_wbc/TWIST-dev/motion_data/AMASS/KIT_572_dance_chacha11_stageii.npz"
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male2MartialArtsPunches_c3d/E1_-__Jab_left_stageii.npz",
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male1Running_c3d/Run_C24_-_quick_side_step_left_stageii.npz",
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2",
                 "booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1", 
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1",
                 "pnd_adam_lite", "openloong", "tienkung", "fourier_gr3", "ultra"],
        default="unitree_g1",
    )
    
    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )

    parser.add_argument(
        "--record_video",
        default=False,
        action="store_true",
        help="Record the video.",
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default=None,
    )

    parser.add_argument(
        "--rate_limit",
        default=False,
        action="store_true",
        help="Limit the rate of the retargeted robot motion to keep the same as the human motion.",
    )

    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run retargeting without opening the realtime viewer.",
    )

    parser.add_argument(
        "--show_skeleton",
        action="store_true",
        default=False,
        help="Overlay raw and scaled SMPL-X skeletons in the realtime viewer.",
    )

    parser.add_argument(
        "--show_robot_body_name",
        action="store_true",
        default=False,
        help="Label robot body frames referenced by the IK JSON.",
    )

    parser.add_argument(
        "--show_human_body_name",
        action="store_true",
        default=False,
        help="Label scaled human target frames.",
    )

    parser.add_argument(
        "--hot_reload_ik_config",
        action="store_true",
        default=False,
        help="Reload the IK JSON when it changes while the viewer is running.",
    )

    args = parser.parse_args()


    SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"
    
    
    # Load SMPLX trajectory
    smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(
        args.smplx_file, SMPLX_FOLDER
    )
    
    # align fps
    tgt_fps = 30
    smplx_data_frames, aligned_fps = get_smplx_data_offline_fast(smplx_data, body_model, smplx_output, tgt_fps=tgt_fps)
    
   
    # Initialize the retargeting system
    retargeter = GMR(
        actual_human_height=actual_human_height,
        src_human="smplx",
        tgt_robot=args.robot,
    )

    skeleton_metadata = make_smplx_skeleton_metadata(body_model) if args.show_skeleton else None
    gmr_robot_body_names = get_gmr_robot_body_names(retargeter)
    robot_frame_options = make_robot_frame_overlay_options(
        show_skeleton=args.show_skeleton,
        show_robot_body_name=args.show_robot_body_name,
    )

    robot_motion_viewer = None
    if not args.headless:
        video_path = args.video_path
        if video_path is None:
            video_path = f"videos/{args.robot}_{args.smplx_file.split('/')[-1].split('.')[0]}.mp4"
        robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                                motion_fps=aligned_fps,
                                                transparent_robot=0,
                                                record_video=args.record_video,
                                                video_path=video_path,)
    
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []
    
    print(f"mocap_frame_rate: {aligned_fps}")
    pbar = tqdm(total=len(smplx_data_frames), desc="Retargeting")

    i = 0

    while True:
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
        
        pbar.update(1)

        # Update task targets.
        smplx_data = smplx_data_frames[i]

        if args.hot_reload_ik_config:
            try:
                if retargeter.reload_ik_config_if_changed():
                    gmr_robot_body_names = get_gmr_robot_body_names(retargeter)
                    print(f"[GMR] Hot reloaded IK config: {retargeter.ik_config_path}")
            except Exception as exc:
                print(f"[GMR] IK config hot reload skipped: {exc}")

        # retarget
        qpos = retargeter.retarget(smplx_data)

        # visualize
        if robot_motion_viewer is not None:
            human_skeletons = None
            if args.show_skeleton:
                if skeleton_metadata is None:
                    raise RuntimeError("--show_skeleton requires SMPL-X skeleton metadata")
                human_skeletons = make_human_skeleton_payloads(
                    smplx_data,
                    retargeter.scaled_human_data,
                    skeleton_metadata,
                )

            robot_motion_viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retargeter.scaled_human_data,
                robot_body_names=gmr_robot_body_names,
                show_robot_body_name=robot_frame_options["show_robot_body_name"],
                robot_frame_scale=robot_frame_options["robot_frame_scale"],
                human_skeletons=human_skeletons,
                show_human_body_name=args.show_human_body_name,
                rate_limit=args.rate_limit,
                follow_camera=True,
            )

        if args.loop:
            i = (i + 1) % len(smplx_data_frames)
        else:
            i += 1
            if i >= len(smplx_data_frames):
                break

        if args.save_path is not None:
            qpos_list.append(qpos)
            
    if args.save_path is not None:
        import pickle
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # save from wxyz to xyzw
        root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])
        local_body_pos = None
        body_names = None
        
        motion_data = {
            "fps": aligned_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved to {args.save_path}")
            
      
    
    pbar.close()

    if robot_motion_viewer is not None:
        robot_motion_viewer.close()
