# GMR Human Skeleton Overlay Design

## Goal

Add an optional GMR viewer overlay that shows both the original human skeleton frame and the scaled/offset human skeleton frame during retargeting. The overlay is enabled from command-line arguments and uses small spheres for joints plus connector lines/capsules for bones.

## Scope

The first implementation targets the existing realtime GMR scripts, especially `scripts/bvh_to_robot.py`, while keeping the viewer API general enough for SMPLX scripts to use the same drawing path later. Existing robot rendering, IK target frame drawing, video recording, and headless retargeting behavior should remain unchanged when the new flag is not provided.

## User-Facing Behavior

Add a command-line flag:

```bash
--show_skeleton
```

When omitted, behavior is unchanged. When provided, the viewer draws two overlapping skeletons:

- Raw input skeleton: the frame data before `retargeter.retarget(...)`.
- Scaled skeleton: `retargeter.scaled_human_data` after GMR scaling, offsets, and ground adjustment.

Both skeletons use the same world coordinates and no default side offset. They are distinguished only by color and alpha.

## Data Flow

For BVH input:

1. `load_bvh_file(...)` should expose skeleton hierarchy metadata derived from the parsed BVH: ordered bone names and parent indices.
2. `scripts/bvh_to_robot.py` keeps `smplx_data = lafan1_data_frames[i]` as the raw frame.
3. After `qpos = retargeter.retarget(smplx_data)`, the script passes both `smplx_data` and `retargeter.scaled_human_data` into `RobotMotionViewer.step(...)` only when `--show_skeleton` is enabled.
4. The viewer draws each skeleton from the provided motion dictionaries and hierarchy metadata.

For future SMPLX use, the same viewer API can accept joint names and parent indices from `body_model.parents` and `JOINT_NAMES`.

## Viewer API

Extend `RobotMotionViewer.step(...)` with this optional argument:

```python
human_skeletons=None
```

Each skeleton entry contains:

```python
{
    "motion_data": raw_or_scaled_frame,
    "joint_names": joint_names,
    "parents": parent_indices,
    "rgba": [r, g, b, a],
    "joint_radius": 0.018,
    "bone_width": 0.01,
    "pos_offset": np.array([0.0, 0.0, 0.0]),
}
```

`motion_data` keeps the existing GMR human data format:

```python
{"joint_name": (position_xyz, quaternion_wxyz)}
```

The viewer should skip a bone if either the child or parent joint is absent from `motion_data`. This matters because GMR configs can add helper joints such as `LeftFootMod` and `RightFootMod` that are not part of the original BVH hierarchy.

## Rendering

Use MuJoCo user scene geoms:

- `mjGEOM_SPHERE` for joints.
- `mjv_connector(...)` with a thin capsule or line-style geom for parent-child bones.

The overlay should clear and redraw through `viewer.user_scn` together with the existing target frame drawing. Skeleton drawing should not require enabling MuJoCo's right-side UI.

Recommended colors:

- Raw skeleton: orange, semi-transparent.
- Scaled skeleton: cyan/blue, more opaque.

## Error Handling

If `--show_skeleton` is requested without hierarchy metadata, fail early with a clear script-level error instead of silently drawing incomplete output. Missing individual joints inside a frame are skipped during drawing.

## Testing

Add focused tests for non-rendering helper behavior:

- BVH loader returns frame data plus stable `joint_names` and `parents`.
- Skeleton draw preparation skips missing joints and only attempts valid parent-child bones.
- `--show_skeleton` remains opt-in and does not affect headless mode.

Manual verification:

```bash
python scripts/bvh_to_robot.py --bvh_file <path> --format nokov --robot ultra --show_skeleton --rate_limit
```

Expected result: robot remains visible; raw and scaled human skeletons overlap, with visible colored joints and bones.
