import argparse
import math
import pathlib
import pickle
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
ULTRA_MJCF_PATH = REPO_ROOT / "assets" / "ultra_GameYaw" / "mjcf" / "ultra_game_yaw.xml"
ULTRA_URDF_PATH = REPO_ROOT / "assets" / "ultra_GameYaw" / "urdf" / "ultra0523Yaw.urdf"


@dataclass(frozen=True)
class JointLimit:
    name: str
    lower: float
    upper: float
    velocity: float | None


class NumpyCompatUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith("numpy._core"):
            module = module.replace("numpy._core", "numpy.core", 1)
        return super().find_class(module, name)


def _as_float_pair(value):
    if not value:
        return None
    parts = value.split()
    if len(parts) != 2:
        return None
    return float(parts[0]), float(parts[1])


def _read_mjcf_actuator_joint_order(mjcf_path):
    root = ET.parse(mjcf_path).getroot()
    joint_names = []
    for actuator in root.findall("./actuator/position"):
        joint_name = actuator.attrib.get("joint")
        if joint_name:
            joint_names.append(joint_name)
    if not joint_names:
        raise ValueError(f"No position actuators with joint names found in {mjcf_path}")
    return joint_names


def _read_mjcf_joint_ranges(mjcf_path):
    root = ET.parse(mjcf_path).getroot()
    ranges = {}
    for joint in root.findall(".//joint"):
        joint_name = joint.attrib.get("name")
        joint_range = _as_float_pair(joint.attrib.get("range"))
        if joint_name and joint_range:
            ranges[joint_name] = joint_range
    return ranges


def _read_urdf_joint_limits(urdf_path):
    root = ET.parse(urdf_path).getroot()
    limits = {}
    for joint in root.findall("joint"):
        joint_name = joint.attrib.get("name")
        limit = joint.find("limit")
        if joint_name is None or limit is None:
            continue
        lower = limit.attrib.get("lower")
        upper = limit.attrib.get("upper")
        velocity = limit.attrib.get("velocity")
        limits[joint_name] = {
            "lower": float(lower) if lower is not None else None,
            "upper": float(upper) if upper is not None else None,
            "velocity": float(velocity) if velocity is not None else None,
        }
    return limits


def load_ultra_joint_limits(mjcf_path=ULTRA_MJCF_PATH, urdf_path=ULTRA_URDF_PATH):
    """Load ultra joint limits in the same order as the MJCF position actuators."""
    joint_order = _read_mjcf_actuator_joint_order(mjcf_path)
    mjcf_ranges = _read_mjcf_joint_ranges(mjcf_path)
    urdf_limits = _read_urdf_joint_limits(urdf_path)

    joint_limits = []
    for joint_name in joint_order:
        urdf_limit = urdf_limits.get(joint_name, {})
        mjcf_range = mjcf_ranges.get(joint_name)
        lower = urdf_limit.get("lower")
        upper = urdf_limit.get("upper")
        if lower is None or upper is None:
            if mjcf_range is None:
                raise ValueError(f"No position limits found for {joint_name}")
            lower, upper = mjcf_range
        joint_limits.append(
            JointLimit(
                name=joint_name,
                lower=float(lower),
                upper=float(upper),
                velocity=urdf_limit.get("velocity"),
            )
        )
    return joint_limits


def compute_joint_velocities(dof_pos, fps):
    dof_pos = np.asarray(dof_pos, dtype=float)
    if dof_pos.ndim != 2:
        raise ValueError(f"dof_pos must be 2D, got shape {dof_pos.shape}")
    if dof_pos.shape[0] < 2:
        return np.zeros_like(dof_pos)
    return np.gradient(dof_pos, 1.0 / float(fps), axis=0, edge_order=1)


def _yaw_from_xyzw(root_rot_xyzw):
    quat = np.asarray(root_rot_xyzw, dtype=float)
    if quat.ndim != 2 or quat.shape[1] != 4:
        raise ValueError(f"root_rot must have shape (N, 4), got {quat.shape}")
    x = quat[:, 0]
    y = quat[:, 1]
    z = quat[:, 2]
    w = quat[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compute_base_velocities(root_pos, root_rot_xyzw, fps):
    root_pos = np.asarray(root_pos, dtype=float)
    if root_pos.ndim != 2 or root_pos.shape[1] < 2:
        raise ValueError(f"root_pos must have shape (N, >=2), got {root_pos.shape}")

    if root_pos.shape[0] < 2:
        return {
            "x": np.zeros(root_pos.shape[0]),
            "y": np.zeros(root_pos.shape[0]),
            "yaw": np.zeros(root_pos.shape[0]),
        }

    dt = 1.0 / float(fps)
    yaw = np.unwrap(_yaw_from_xyzw(root_rot_xyzw))
    return {
        "x": np.gradient(root_pos[:, 0], dt, edge_order=1),
        "y": np.gradient(root_pos[:, 1], dt, edge_order=1),
        "yaw": np.gradient(yaw, dt, edge_order=1),
    }


def load_motion(input_file):
    with open(input_file, "rb") as f:
        motion = NumpyCompatUnpickler(f).load()

    required_keys = ("fps", "root_pos", "root_rot", "dof_pos")
    missing = [key for key in required_keys if key not in motion]
    if missing:
        raise KeyError(f"Motion file is missing required keys: {missing}")
    return motion


def _make_output_path(input_file, suffix, output_file=None):
    input_file = pathlib.Path(input_file)
    if output_file is None:
        return input_file.with_name(f"{input_file.stem}_{suffix}.png")

    output_file = pathlib.Path(output_file)
    if suffix:
        return output_file.with_name(f"{output_file.stem}_{suffix}{output_file.suffix or '.png'}")
    return output_file


def _prepare_axes(count, title, columns=2):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = math.ceil(count / columns)
    fig, axes = plt.subplots(rows, columns, figsize=(7.0 * columns, 2.2 * rows), sharex=True)
    axes = np.atleast_1d(axes).reshape(-1)
    fig.suptitle(title)
    for axis in axes[count:]:
        axis.set_visible(False)
    return fig, axes


def plot_joint_positions(time_s, dof_pos, joint_limits, output_path):
    dof_pos = np.asarray(dof_pos, dtype=float)
    fig, axes = _prepare_axes(len(joint_limits), "Joint Position Limits", columns=2)
    for idx, joint_limit in enumerate(joint_limits):
        axis = axes[idx]
        axis.plot(time_s, dof_pos[:, idx], label="position")
        axis.axhline(joint_limit.upper, color="tab:red", linestyle="--", linewidth=0.9, label="upper")
        axis.axhline(joint_limit.lower, color="tab:red", linestyle="--", linewidth=0.9, label="lower")
        axis.set_title(joint_limit.name, fontsize=9)
        axis.set_ylabel("rad")
        axis.grid(True, alpha=0.25)
    axes[0].legend(loc="best", fontsize=8)
    axes[min(len(joint_limits) - 1, len(axes) - 1)].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    return output_path


def plot_joint_velocities(time_s, dof_vel, joint_limits, output_path):
    dof_vel = np.asarray(dof_vel, dtype=float)
    fig, axes = _prepare_axes(len(joint_limits), "Joint Velocity Limits", columns=2)
    for idx, joint_limit in enumerate(joint_limits):
        axis = axes[idx]
        axis.plot(time_s, dof_vel[:, idx], label="velocity")
        if joint_limit.velocity is not None:
            axis.axhline(joint_limit.velocity, color="tab:red", linestyle="--", linewidth=0.9, label="+limit")
            axis.axhline(-joint_limit.velocity, color="tab:red", linestyle="--", linewidth=0.9, label="-limit")
        axis.set_title(joint_limit.name, fontsize=9)
        axis.set_ylabel("rad/s")
        axis.grid(True, alpha=0.25)
    axes[0].legend(loc="best", fontsize=8)
    axes[min(len(joint_limits) - 1, len(axes) - 1)].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    return output_path


def plot_base_velocities(time_s, base_vel, output_path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    series = [
        ("base x linear velocity", base_vel["x"], "m/s"),
        ("base y linear velocity", base_vel["y"], "m/s"),
        ("base yaw velocity", base_vel["yaw"], "rad/s"),
    ]
    for axis, (title, values, ylabel) in zip(axes, series):
        axis.plot(time_s, values)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    return output_path


def validate_motion_shape(motion, joint_limits):
    dof_pos = np.asarray(motion["dof_pos"])
    if dof_pos.ndim != 2:
        raise ValueError(f"dof_pos must be 2D, got shape {dof_pos.shape}")
    if dof_pos.shape[1] != len(joint_limits):
        names = ", ".join(joint.name for joint in joint_limits)
        raise ValueError(
            f"dof_pos has {dof_pos.shape[1]} joints but ultra asset has {len(joint_limits)} joints: {names}"
        )


def save_selected_plots(args):
    motion = load_motion(args.input_file)
    joint_limits = load_ultra_joint_limits()
    validate_motion_shape(motion, joint_limits)

    fps = float(motion["fps"])
    dof_pos = np.asarray(motion["dof_pos"], dtype=float)
    root_pos = np.asarray(motion["root_pos"], dtype=float)
    root_rot = np.asarray(motion["root_rot"], dtype=float)
    time_s = np.arange(dof_pos.shape[0], dtype=float) / fps

    selected_count = sum([args.plot_joints_pos, args.plot_joints_vel, args.plot_base_vel])
    output_paths = []
    if args.plot_joints_pos:
        suffix = "joint_pos" if selected_count > 1 else ""
        output_paths.append(plot_joint_positions(time_s, dof_pos, joint_limits, _make_output_path(args.input_file, suffix, args.output_file)))
    if args.plot_joints_vel:
        suffix = "joint_vel" if selected_count > 1 else ""
        dof_vel = compute_joint_velocities(dof_pos, fps)
        output_paths.append(plot_joint_velocities(time_s, dof_vel, joint_limits, _make_output_path(args.input_file, suffix, args.output_file)))
    if args.plot_base_vel:
        suffix = "base_vel" if selected_count > 1 else ""
        base_vel = compute_base_velocities(root_pos, root_rot, fps)
        output_paths.append(plot_base_velocities(time_s, base_vel, _make_output_path(args.input_file, suffix, args.output_file)))
    return output_paths


def parse_args():
    parser = argparse.ArgumentParser(description="Plot GMR robot motion limits for ultra.")
    parser.add_argument("--input_file", required=True, help="GMR output pickle file.")
    parser.add_argument("--plot_joints_pos", action="store_true", help="Plot joint positions with position limits.")
    parser.add_argument("--plot_joints_vel", action="store_true", help="Plot joint velocities with velocity limits.")
    parser.add_argument("--plot_base_vel", action="store_true", help="Plot base x/y linear velocity and yaw velocity.")
    parser.add_argument("--output_file", default=None, help="Output PNG path. With multiple plot groups, suffixes are added.")
    return parser.parse_args()


def main():
    args = parse_args()
    if not (args.plot_joints_pos or args.plot_joints_vel or args.plot_base_vel):
        args.plot_joints_pos = True
        args.plot_joints_vel = True
        args.plot_base_vel = True

    output_paths = save_selected_plots(args)
    for output_path in output_paths:
        print(f"Saved {output_path}")


if __name__ == "__main__":
    main()
