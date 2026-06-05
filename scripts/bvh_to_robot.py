import argparse
import pathlib
import time
from rich import print
from tqdm import tqdm
import os
import numpy as np


def make_scaled_skeleton_metadata(scaled_frame, metadata):
    joint_names = list(metadata["joint_names"])
    parents = list(metadata["parents"])
    joint_name_to_index = {joint_name: idx for idx, joint_name in enumerate(joint_names)}
    helper_parent_names = {
        "LeftFootMod": "LeftLeg",
        "RightFootMod": "RightLeg",
    }

    for helper_name, parent_name in helper_parent_names.items():
        if helper_name not in scaled_frame or helper_name in joint_name_to_index:
            continue
        if parent_name not in joint_name_to_index:
            continue
        joint_name_to_index[helper_name] = len(joint_names)
        joint_names.append(helper_name)
        parents.append(joint_name_to_index[parent_name])

    return {
        "joint_names": joint_names,
        "parents": np.asarray(parents, dtype=int),
    }


def make_raw_skeleton_pos_offset(raw_frame, scaled_frame, metadata):
    root_name = metadata["joint_names"][0]
    if root_name not in raw_frame or root_name not in scaled_frame:
        return np.zeros(3)
    return np.asarray(scaled_frame[root_name][0]) - np.asarray(raw_frame[root_name][0])


def make_human_skeleton_payloads(raw_frame, scaled_frame, metadata):
    joint_names = metadata["joint_names"]
    parents = metadata["parents"]
    scaled_metadata = make_scaled_skeleton_metadata(scaled_frame, metadata)
    raw_pos_offset = make_raw_skeleton_pos_offset(raw_frame, scaled_frame, metadata)
    return [
        {
            "motion_data": raw_frame,
            "joint_names": joint_names,
            "parents": parents,
            "rgba": [1.0, 0.45, 0.05, 0.45],
            "joint_radius": 0.016,
            "bone_width": 0.008,
            "pos_offset": raw_pos_offset,
            "show_frames": True,
            "frame_scale": 0.06,
            "frame_arrow_width": 0.003,
        },
        {
            "motion_data": scaled_frame,
            "joint_names": scaled_metadata["joint_names"],
            "parents": scaled_metadata["parents"],
            "rgba": [0.05, 0.75, 1.0, 0.75],
            "joint_radius": 0.018,
            "bone_width": 0.01,
            "connect_to_nearest_available": True,
        },
    ]


def make_robot_frame_overlay_options(show_skeleton=False, show_robot_body_name=False):
    return {
        "show_robot_body_name": show_robot_body_name,
        "robot_frame_scale": 0.12 if show_skeleton else 0.06,
    }


if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bvh_file",
        help="BVH motion file to load.",
        required=True,
        type=str,
    )
    
    parser.add_argument(
        "--format",
        choices=["lafan1", "nokov"],
        default="lafan1",
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "fourier_n1", "engineai_pm01", "pal_talos", "ultra"],
        default="unitree_g1",
    )
    
    
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )

    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
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
        help="Overlay raw and scaled human skeletons in the realtime viewer.",
    )

    parser.add_argument(
        "--show_robot_body_name",
        action="store_true",
        default=False,
        help="Label robot body frames referenced by the IK JSON.",
    )

    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--motion_fps",
        default=30,
        type=int,
    )
    
    args = parser.parse_args()

    from general_motion_retargeting import GeneralMotionRetargeting as GMR
    from general_motion_retargeting import RobotMotionViewer
    from general_motion_retargeting.robot_motion_viewer import get_gmr_robot_body_names
    from general_motion_retargeting.utils.lafan1 import load_bvh_file
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []

    
    # Load SMPLX trajectory
    if args.show_skeleton:
        lafan1_data_frames, actual_human_height, skeleton_metadata = load_bvh_file(
            args.bvh_file,
            format=args.format,
            return_metadata=True,
        )
    else:
        lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file, format=args.format)
        skeleton_metadata = None
    
    
    # Initialize the retargeting system
    retargeter = GMR(
        src_human=f"bvh_{args.format}",
        tgt_robot=args.robot,
        actual_human_height=actual_human_height,
    )

    motion_fps = args.motion_fps
    
    gmr_robot_body_names = get_gmr_robot_body_names(retargeter)
    robot_frame_options = make_robot_frame_overlay_options(
        show_skeleton=args.show_skeleton,
        show_robot_body_name=args.show_robot_body_name,
    )

    robot_motion_viewer = None
    if not args.headless:
        robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                                motion_fps=motion_fps,
                                                transparent_robot=0,
                                                record_video=args.record_video,
                                                video_path=args.video_path,
                                                # video_width=2080,
                                                # video_height=1170
                                                )
    
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    print(f"mocap_frame_rate: {motion_fps}")
    
    # Create tqdm progress bar for the total number of frames
    pbar = tqdm(total=len(lafan1_data_frames), desc="Retargeting")
    
    # Start the viewer
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
            
        # Update progress bar
        pbar.update(1)

        # Update task targets.
        smplx_data = lafan1_data_frames[i]

        # retarget
        qpos = retargeter.retarget(smplx_data)
        

        # visualize
        if robot_motion_viewer is not None:
            human_skeletons = None
            if args.show_skeleton:
                if skeleton_metadata is None:
                    raise RuntimeError("--show_skeleton requires BVH skeleton metadata")
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
                rate_limit=args.rate_limit,
                follow_camera=True,
                # human_pos_offset=np.array([0.0, 0.0, 0.0])
            )

        if args.loop:
            i = (i + 1) % len(lafan1_data_frames)
        else:
            i += 1
            if i >= len(lafan1_data_frames):
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
            "fps": motion_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved to {args.save_path}")

    # Close progress bar
    pbar.close()
    
    if robot_motion_viewer is not None:
        robot_motion_viewer.close()
       
