import numpy as np
import pytest
import sys
import types

sys.modules.setdefault("mink", types.SimpleNamespace())

from general_motion_retargeting.robot_motion_viewer import (
    build_skeleton_bone_segments,
    build_skeleton_joint_positions,
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
