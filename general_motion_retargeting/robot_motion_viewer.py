import os
import time
import mujoco as mj
import mujoco.viewer as mjv
import imageio
from scipy.spatial.transform import Rotation as R
from general_motion_retargeting import ROBOT_XML_DICT, ROBOT_BASE_DICT, VIEWER_CAM_DISTANCE_DICT
from loop_rate_limiters import RateLimiter
import numpy as np
from rich import print


def draw_frame(
    pos,
    mat,
    v,
    size,
    joint_name=None,
    orientation_correction=R.from_euler("xyz", [0, 0, 0]),
    pos_offset=np.array([0, 0, 0]),
    arrow_width=0.005,
):
    rgba_list = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
    for i in range(3):
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_ARROW,
            size=[0.01, 0.01, 0.01],
            pos=pos + pos_offset,
            mat=mat.flatten(),
            rgba=rgba_list[i],
        )
        if joint_name is not None:
            geom.label = joint_name  # 这里赋名字
        fix = orientation_correction.as_matrix()
        mj.mjv_connector(
            v.user_scn.geoms[v.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=arrow_width,
            from_=pos + pos_offset,
            to=pos + pos_offset + size * (mat @ fix)[:, i],
        )
        v.user_scn.ngeom += 1


def build_skeleton_joint_positions(motion_data, joint_names, pos_offset=None):
    if pos_offset is None:
        pos_offset = np.zeros(3)
    pos_offset = np.asarray(pos_offset)

    joint_positions = {}
    for joint_name in joint_names:
        if joint_name not in motion_data:
            continue
        pos, _ = motion_data[joint_name]
        joint_positions[joint_name] = np.asarray(pos) + pos_offset
    return joint_positions


def build_skeleton_bone_segments(
    motion_data,
    joint_names,
    parents,
    pos_offset=None,
    connect_to_nearest_available=False,
):
    if len(joint_names) != len(parents):
        raise ValueError("joint_names and parents must have the same length")

    joint_positions = build_skeleton_joint_positions(motion_data, joint_names, pos_offset)
    segments = []
    for child_index, parent_index in enumerate(parents):
        if parent_index < 0:
            continue
        if connect_to_nearest_available:
            while parent_index >= 0 and joint_names[int(parent_index)] not in joint_positions:
                parent_index = parents[int(parent_index)]
            if parent_index < 0:
                continue
        parent_name = joint_names[int(parent_index)]
        child_name = joint_names[child_index]
        if parent_name not in joint_positions or child_name not in joint_positions:
            continue
        segments.append((parent_name, child_name, joint_positions[parent_name], joint_positions[child_name]))
    return segments


def normalize_skeleton_payload(payload):
    normalized = dict(payload)
    normalized.setdefault("joint_radius", 0.018)
    normalized.setdefault("bone_width", 0.01)
    normalized.setdefault("pos_offset", np.zeros(3))
    normalized.setdefault("connect_to_nearest_available", False)
    normalized.setdefault("show_frames", False)
    normalized.setdefault("frame_scale", 0.05)
    normalized.setdefault("frame_arrow_width", 0.003)
    normalized["pos_offset"] = np.asarray(normalized["pos_offset"])
    return normalized


def _draw_skeleton(
    v,
    motion_data,
    joint_names,
    parents,
    rgba,
    joint_radius=0.018,
    bone_width=0.01,
    pos_offset=None,
    connect_to_nearest_available=False,
    show_frames=False,
    frame_scale=0.05,
    frame_arrow_width=0.003,
):
    joint_positions = build_skeleton_joint_positions(motion_data, joint_names, pos_offset)
    if pos_offset is None:
        pos_offset = np.zeros(3)
    pos_offset = np.asarray(pos_offset)

    for pos in joint_positions.values():
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_SPHERE,
            size=[joint_radius, 0.0, 0.0],
            pos=pos,
            mat=np.eye(3).flatten(),
            rgba=rgba,
        )
        v.user_scn.ngeom += 1

    if show_frames:
        for joint_name in joint_names:
            if joint_name not in motion_data:
                continue
            pos, rot = motion_data[joint_name]
            draw_frame(
                np.asarray(pos),
                R.from_quat(rot, scalar_first=True).as_matrix(),
                v,
                frame_scale,
                pos_offset=pos_offset,
                arrow_width=frame_arrow_width,
            )

    for _, _, parent_pos, child_pos in build_skeleton_bone_segments(
        motion_data,
        joint_names,
        parents,
        pos_offset,
        connect_to_nearest_available=connect_to_nearest_available,
    ):
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_CAPSULE,
            size=[bone_width, 0.0, 0.0],
            pos=np.zeros(3),
            mat=np.eye(3).flatten(),
            rgba=rgba,
        )
        mj.mjv_connector(
            geom,
            type=mj.mjtGeom.mjGEOM_CAPSULE,
            width=bone_width,
            from_=parent_pos,
            to=child_pos,
        )
        v.user_scn.ngeom += 1


def get_gmr_robot_body_names(retargeter):
    """Return robot bodies that are enabled and weighted in GMR IK tasks."""
    robot_body_names = []
    task_tables = (
        (retargeter.use_ik_match_table1, retargeter.ik_match_table1),
        (retargeter.use_ik_match_table2, retargeter.ik_match_table2),
    )

    for use_table, ik_match_table in task_tables:
        if not use_table:
            continue
        for robot_body_name, entry in ik_match_table.items():
            _, pos_weight, rot_weight, _, _ = entry
            if (pos_weight != 0 or rot_weight != 0) and robot_body_name not in robot_body_names:
                robot_body_names.append(robot_body_name)

    return robot_body_names


class RobotMotionViewer:
    def __init__(self,
                robot_type,
                camera_follow=True,
                motion_fps=30,
                transparent_robot=0,
                # video recording
                record_video=False,
                video_path=None,
                video_width=640,
                video_height=480,
                keyboard_callback=None,
                ):
        
        self.robot_type = robot_type
        self.xml_path = ROBOT_XML_DICT[robot_type]
        self.model = mj.MjModel.from_xml_path(str(self.xml_path))
        self.data = mj.MjData(self.model)
        self.robot_base = ROBOT_BASE_DICT[robot_type]
        self.viewer_cam_distance = VIEWER_CAM_DISTANCE_DICT[robot_type]
        mj.mj_step(self.model, self.data)
        
        self.motion_fps = motion_fps
        self.rate_limiter = RateLimiter(frequency=self.motion_fps, warn=False)
        self.camera_follow = camera_follow
        self._camera_initialized = False
        self.record_video = record_video


        self.viewer = mjv.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False, 
            key_callback=keyboard_callback
            )      

        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = transparent_robot
        
        if self.record_video:
            assert video_path is not None, "Please provide video path for recording"
            self.video_path = video_path
            video_dir = os.path.dirname(self.video_path)
            
            if not os.path.exists(video_dir):
                os.makedirs(video_dir)
            self.mp4_writer = imageio.get_writer(self.video_path, fps=self.motion_fps)
            print(f"Recording video to {self.video_path}")
            
            # Initialize renderer for video recording
            self.renderer = mj.Renderer(self.model, height=video_height, width=video_width)
        
    def step(self, 
            # robot data
            root_pos, root_rot, dof_pos, 
            # human data
            human_motion_data=None, 
            show_human_body_name=False,
            # scale for human point visualization
            human_point_scale=0.1,
            # human pos offset add for visualization    
            human_pos_offset=np.array([0.0, 0.0, 0]),
            # robot body frames to compare with GMR targets
            robot_body_names=None,
            show_robot_body_name=False,
            robot_frame_scale=0.06,
            human_skeletons=None,
            # rate limit
            rate_limit=True, 
            follow_camera=False,
            ):
        """
        by default visualize robot motion.
        also support visualize human motion by providing human_motion_data, to compare with robot motion.
        
        human_motion_data is a dict of {"human body name": (3d global translation, 3d global rotation)}.
        robot_body_names lists robot body frames to draw from the current MuJoCo state.

        if rate_limit is True, the motion will be visualized at the same rate as the motion data.
        else, the motion will be visualized as fast as possible.
        """
        
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot # quat need to be scalar first! for mujoco
        self.data.qpos[7:] = dof_pos
        
        mj.mj_forward(self.model, self.data)
        
        if follow_camera and not self._camera_initialized:
            self.viewer.cam.lookat = self.data.xpos[self.model.body(self.robot_base).id]
            self.viewer.cam.distance = self.viewer_cam_distance
            self.viewer.cam.elevation = -10  # 正面视角，轻微向下看
            # self.viewer.cam.azimuth = 180    # 正面朝向机器人
            self._camera_initialized = True
        
        if human_motion_data is not None or robot_body_names is not None or human_skeletons is not None:
            # Clean custom geometry
            self.viewer.user_scn.ngeom = 0

        if human_motion_data is not None:
            # Draw the task targets for reference
            for human_body_name, (pos, rot) in human_motion_data.items():
                draw_frame(
                    pos,
                    R.from_quat(rot, scalar_first=True).as_matrix(),
                    self.viewer,
                    human_point_scale,
                    pos_offset=human_pos_offset,
                    joint_name=human_body_name if show_human_body_name else None
                    )

        if robot_body_names is not None:
            # Draw robot link frames used by GMR tasks from the current MuJoCo pose.
            for robot_body_name in robot_body_names:
                body_id = self.model.body(robot_body_name).id
                draw_frame(
                    self.data.xpos[body_id],
                    self.data.xmat[body_id].reshape(3, 3),
                    self.viewer,
                    robot_frame_scale,
                    joint_name=robot_body_name if show_robot_body_name else None,
                    arrow_width=0.003,
                    )

        if human_skeletons is not None:
            for skeleton_payload in human_skeletons:
                payload = normalize_skeleton_payload(skeleton_payload)
                _draw_skeleton(
                    self.viewer,
                    motion_data=payload["motion_data"],
                    joint_names=payload["joint_names"],
                    parents=payload["parents"],
                    rgba=payload["rgba"],
                    joint_radius=payload["joint_radius"],
                    bone_width=payload["bone_width"],
                    pos_offset=payload["pos_offset"],
                    connect_to_nearest_available=payload["connect_to_nearest_available"],
                    show_frames=payload["show_frames"],
                    frame_scale=payload["frame_scale"],
                    frame_arrow_width=payload["frame_arrow_width"],
                )

        self.viewer.sync()
        if rate_limit is True:
            self.rate_limiter.sleep()

        if self.record_video:
            # Use renderer for proper offscreen rendering
            self.renderer.update_scene(self.data, camera=self.viewer.cam)
            img = self.renderer.render()
            self.mp4_writer.append_data(img)
    
    def close(self):
        self.viewer.close()
        time.sleep(0.5)
        if self.record_video:
            self.mp4_writer.close()
            print(f"Video saved to {self.video_path}")
