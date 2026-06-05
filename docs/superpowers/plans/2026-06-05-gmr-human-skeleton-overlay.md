# GMR Human Skeleton Overlay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use $superpower-subagents (recommended) or $superpower-executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking via update_plan.

**Goal:** Add an opt-in `--show_skeleton` overlay that draws raw and scaled human skeletons as overlapping small spheres plus bones during BVH GMR retargeting.

**Architecture:** Keep drawing responsibilities in `RobotMotionViewer`; scripts only assemble optional skeleton payloads from raw frame data, scaled frame data, and BVH hierarchy metadata. Add small pure helpers for skeleton data validation/preparation so behavior can be tested without launching MuJoCo's viewer.

**Tech Stack:** Python, NumPy, MuJoCo user scene geoms, pytest.

---

## File Structure

- Modify `general_motion_retargeting/utils/lafan1.py`: optionally return BVH hierarchy metadata while preserving the current two-value return by default.
- Modify `general_motion_retargeting/robot_motion_viewer.py`: add pure skeleton helper functions and draw skeleton payloads in `RobotMotionViewer.step(...)`.
- Modify `scripts/bvh_to_robot.py`: parse `--show_skeleton`, request hierarchy metadata from the loader, and pass raw/scaled skeleton overlays into the viewer.
- Create `tests/test_robot_motion_viewer_skeleton.py`: test pure helper behavior.

---

### Task 1: Add Test Coverage For Skeleton Segment Preparation

**Files:**
- Create: `tests/test_robot_motion_viewer_skeleton.py`
- Modify: `general_motion_retargeting/robot_motion_viewer.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_robot_motion_viewer_skeleton.py`:

```python
import numpy as np
import pytest

from general_motion_retargeting.robot_motion_viewer import (
    build_skeleton_joint_positions,
    build_skeleton_bone_segments,
)


def test_build_skeleton_joint_positions_applies_offset_and_skips_missing_joint():
    motion_data = {
        "Root": (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
        "Chest": (np.array([0.0, 0.0, 1.5]), np.array([1.0, 0.0, 0.0, 0.0])),
    }

    positions = build_skeleton_joint_positions(
        motion_data,
        ["Root", "Chest", "Missing"],
        pos_offset=np.array([1.0, 0.0, 0.0]),
    )

    assert set(positions) == {"Root", "Chest"}
    np.testing.assert_allclose(positions["Root"], np.array([1.0, 0.0, 1.0]))
    np.testing.assert_allclose(positions["Chest"], np.array([1.0, 0.0, 1.5]))


def test_build_skeleton_bone_segments_skips_missing_parent_or_child():
    motion_data = {
        "Root": (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
        "Chest": (np.array([0.0, 0.0, 1.5]), np.array([1.0, 0.0, 0.0, 0.0])),
    }

    segments = build_skeleton_bone_segments(
        motion_data,
        joint_names=["Root", "Chest", "Head"],
        parents=np.array([-1, 0, 1]),
    )

    assert len(segments) == 1
    parent_name, child_name, parent_pos, child_pos = segments[0]
    assert parent_name == "Root"
    assert child_name == "Chest"
    np.testing.assert_allclose(parent_pos, np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(child_pos, np.array([0.0, 0.0, 1.5]))


def test_build_skeleton_bone_segments_rejects_mismatched_hierarchy_lengths():
    with pytest.raises(ValueError, match="joint_names and parents must have the same length"):
        build_skeleton_bone_segments(
            {},
            joint_names=["Root", "Chest"],
            parents=np.array([-1]),
        )
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: collection/import failure because `build_skeleton_joint_positions` and `build_skeleton_bone_segments` do not exist yet.

- [ ] **Step 3: Add minimal helper implementations**

In `general_motion_retargeting/robot_motion_viewer.py`, add these functions near `draw_frame(...)`:

```python
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


def build_skeleton_bone_segments(motion_data, joint_names, parents, pos_offset=None):
    if len(joint_names) != len(parents):
        raise ValueError("joint_names and parents must have the same length")

    joint_positions = build_skeleton_joint_positions(motion_data, joint_names, pos_offset)
    segments = []
    for child_index, parent_index in enumerate(parents):
        if parent_index < 0:
            continue
        parent_name = joint_names[int(parent_index)]
        child_name = joint_names[child_index]
        if parent_name not in joint_positions or child_name not in joint_positions:
            continue
        segments.append((parent_name, child_name, joint_positions[parent_name], joint_positions[child_name]))
    return segments
```

- [ ] **Step 4: Run tests to verify they pass**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: `3 passed`.

- [ ] **Step 5: Commit**

```bash
git add tests/test_robot_motion_viewer_skeleton.py general_motion_retargeting/robot_motion_viewer.py
git commit -m "test: cover skeleton overlay helpers"
```

---

### Task 2: Draw Skeleton Payloads In RobotMotionViewer

**Files:**
- Modify: `general_motion_retargeting/robot_motion_viewer.py`
- Test: `tests/test_robot_motion_viewer_skeleton.py`

- [ ] **Step 1: Write the failing test for payload defaults**

Append to `tests/test_robot_motion_viewer_skeleton.py`:

```python
from general_motion_retargeting.robot_motion_viewer import normalize_skeleton_payload


def test_normalize_skeleton_payload_fills_defaults():
    payload = normalize_skeleton_payload(
        {
            "motion_data": {},
            "joint_names": ["Root"],
            "parents": np.array([-1]),
            "rgba": [1.0, 0.0, 0.0, 0.5],
        }
    )

    assert payload["joint_radius"] == pytest.approx(0.018)
    assert payload["bone_width"] == pytest.approx(0.01)
    np.testing.assert_allclose(payload["pos_offset"], np.zeros(3))
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: import failure because `normalize_skeleton_payload` does not exist yet.

- [ ] **Step 3: Add payload normalization and drawing functions**

In `general_motion_retargeting/robot_motion_viewer.py`, add:

```python
def normalize_skeleton_payload(payload):
    normalized = dict(payload)
    normalized.setdefault("joint_radius", 0.018)
    normalized.setdefault("bone_width", 0.01)
    normalized.setdefault("pos_offset", np.zeros(3))
    normalized["pos_offset"] = np.asarray(normalized["pos_offset"])
    return normalized


def _draw_skeleton(v, motion_data, joint_names, parents, rgba, joint_radius=0.018, bone_width=0.01, pos_offset=None):
    joint_positions = build_skeleton_joint_positions(motion_data, joint_names, pos_offset)
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

    for _, _, parent_pos, child_pos in build_skeleton_bone_segments(motion_data, joint_names, parents, pos_offset):
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
```

Extend `RobotMotionViewer.step(...)` with `human_skeletons=None`, include it in the custom geometry clearing condition, and draw:

```python
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
                )
```

- [ ] **Step 4: Run tests to verify they pass**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: `4 passed`.

- [ ] **Step 5: Commit**

```bash
git add tests/test_robot_motion_viewer_skeleton.py general_motion_retargeting/robot_motion_viewer.py
git commit -m "feat: draw human skeleton overlays"
```

---

### Task 3: Expose BVH Hierarchy Metadata

**Files:**
- Modify: `general_motion_retargeting/utils/lafan1.py`
- Test: `tests/test_robot_motion_viewer_skeleton.py`

- [ ] **Step 1: Write the failing test for loader API compatibility**

Append to `tests/test_robot_motion_viewer_skeleton.py`:

```python
from unittest.mock import patch

from general_motion_retargeting.utils.lafan1 import load_bvh_file


class FakeAnim:
    def __init__(self):
        self.quats = np.zeros((1, 2, 4))
        self.pos = np.zeros((1, 2, 3))
        self.parents = np.array([-1, 0])
        self.bones = ["Root", "Chest"]


def test_load_bvh_file_can_return_hierarchy_metadata():
    fake_anim = FakeAnim()
    fake_global_data = (
        np.array([[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]]),
        np.array([[[0.0, 0.0, 100.0], [0.0, 0.0, 150.0]]]),
    )

    with patch("general_motion_retargeting.utils.lafan1.read_bvh", return_value=fake_anim), patch(
        "general_motion_retargeting.utils.lafan1.utils.quat_fk", return_value=fake_global_data
    ):
        frames, human_height, metadata = load_bvh_file("fake.bvh", return_metadata=True)

    assert human_height == pytest.approx(1.75)
    assert metadata["joint_names"] == ["Root", "Chest"]
    np.testing.assert_array_equal(metadata["parents"], np.array([-1, 0]))
    assert "Root" in frames[0]
    assert "Chest" in frames[0]
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: `TypeError` because `load_bvh_file(...)` does not accept `return_metadata`.

- [ ] **Step 3: Implement `return_metadata`**

Change the signature and return in `general_motion_retargeting/utils/lafan1.py`:

```python
def load_bvh_file(bvh_file, format="lafan1", return_metadata=False):
```

At the end:

```python
    if return_metadata:
        metadata = {
            "joint_names": list(data.bones),
            "parents": np.asarray(data.parents, dtype=int),
        }
        return frames, human_height, metadata

    return frames, human_height
```

- [ ] **Step 4: Run tests to verify they pass**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: `5 passed`.

- [ ] **Step 5: Commit**

```bash
git add tests/test_robot_motion_viewer_skeleton.py general_motion_retargeting/utils/lafan1.py
git commit -m "feat: expose BVH skeleton hierarchy"
```

---

### Task 4: Add `--show_skeleton` To BVH Retargeting Script

**Files:**
- Modify: `scripts/bvh_to_robot.py`
- Test: `tests/test_robot_motion_viewer_skeleton.py`

- [ ] **Step 1: Write the failing test for payload construction**

Append to `tests/test_robot_motion_viewer_skeleton.py`:

```python
from scripts.bvh_to_robot import make_human_skeleton_payloads


def test_make_human_skeleton_payloads_returns_raw_and_scaled_overlays():
    raw_frame = {
        "Root": (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
    }
    scaled_frame = {
        "Root": (np.array([0.0, 0.0, 1.2]), np.array([1.0, 0.0, 0.0, 0.0])),
    }
    metadata = {"joint_names": ["Root"], "parents": np.array([-1])}

    payloads = make_human_skeleton_payloads(raw_frame, scaled_frame, metadata)

    assert len(payloads) == 2
    assert payloads[0]["motion_data"] is raw_frame
    assert payloads[1]["motion_data"] is scaled_frame
    assert payloads[0]["joint_names"] == ["Root"]
    np.testing.assert_array_equal(payloads[1]["parents"], np.array([-1]))
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: import failure because `make_human_skeleton_payloads` does not exist.

- [ ] **Step 3: Implement script flag and payload builder**

In `scripts/bvh_to_robot.py`, add:

```python
def make_human_skeleton_payloads(raw_frame, scaled_frame, metadata):
    joint_names = metadata["joint_names"]
    parents = metadata["parents"]
    return [
        {
            "motion_data": raw_frame,
            "joint_names": joint_names,
            "parents": parents,
            "rgba": [1.0, 0.45, 0.05, 0.45],
            "joint_radius": 0.016,
            "bone_width": 0.008,
        },
        {
            "motion_data": scaled_frame,
            "joint_names": joint_names,
            "parents": parents,
            "rgba": [0.05, 0.75, 1.0, 0.75],
            "joint_radius": 0.018,
            "bone_width": 0.01,
        },
    ]
```

Add the CLI argument:

```python
    parser.add_argument(
        "--show_skeleton",
        action="store_true",
        default=False,
        help="Overlay raw and scaled human skeletons in the realtime viewer.",
    )
```

Load metadata:

```python
    if args.show_skeleton:
        lafan1_data_frames, actual_human_height, skeleton_metadata = load_bvh_file(
            args.bvh_file, format=args.format, return_metadata=True
        )
    else:
        lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file, format=args.format)
        skeleton_metadata = None
```

Before `robot_motion_viewer.step(...)`:

```python
            human_skeletons = None
            if args.show_skeleton:
                if skeleton_metadata is None:
                    raise RuntimeError("--show_skeleton requires BVH skeleton metadata")
                human_skeletons = make_human_skeleton_payloads(
                    smplx_data,
                    retargeter.scaled_human_data,
                    skeleton_metadata,
                )
```

Pass `human_skeletons=human_skeletons` to `step(...)`.

- [ ] **Step 4: Run tests to verify they pass**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: `6 passed`.

- [ ] **Step 5: Run CLI smoke check**

Run:

```bash
python scripts/bvh_to_robot.py --help | rg -- "--show_skeleton"
```

Expected: the help output contains `--show_skeleton`.

- [ ] **Step 6: Commit**

```bash
git add tests/test_robot_motion_viewer_skeleton.py scripts/bvh_to_robot.py
git commit -m "feat: add BVH skeleton overlay flag"
```

---

### Task 5: Final Verification

**Files:**
- Verify: `general_motion_retargeting/robot_motion_viewer.py`
- Verify: `general_motion_retargeting/utils/lafan1.py`
- Verify: `scripts/bvh_to_robot.py`
- Verify: `tests/test_robot_motion_viewer_skeleton.py`

- [ ] **Step 1: Run focused test suite**

Run:

```bash
pytest tests/test_robot_motion_viewer_skeleton.py -q
```

Expected: `6 passed`.

- [ ] **Step 2: Run CLI help smoke check**

Run:

```bash
python scripts/bvh_to_robot.py --help | rg -- "--show_skeleton"
```

Expected: the command exits 0 and prints the `--show_skeleton` option.

- [ ] **Step 3: Inspect git diff**

Run:

```bash
git diff --stat HEAD
git status --short
```

Expected: no uncommitted changes from this feature; pre-existing user changes may remain in unrelated IK config/doc files.
