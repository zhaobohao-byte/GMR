import numpy as np
import pytest
import sys
import types
from unittest.mock import patch

sys.modules.setdefault("mink", types.SimpleNamespace())

from general_motion_retargeting.robot_motion_viewer import (
    build_skeleton_bone_segments,
    build_skeleton_joint_positions,
    normalize_skeleton_payload,
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


from general_motion_retargeting.utils.lafan1 import load_bvh_file


class FakeAnim:
    def __init__(self):
        self.quats = np.zeros((1, 6, 4))
        self.pos = np.zeros((1, 6, 3))
        self.parents = np.array([-1, 0, 0, 2, 0, 4])
        self.bones = ["Root", "Chest", "LeftFoot", "LeftToe", "RightFoot", "RightToe"]


def test_load_bvh_file_can_return_hierarchy_metadata():
    fake_anim = FakeAnim()
    fake_global_data = (
        np.array(
            [
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                ]
            ]
        ),
        np.array(
            [
                [
                    [0.0, 0.0, 100.0],
                    [0.0, 0.0, 150.0],
                    [0.0, 20.0, 10.0],
                    [0.0, 25.0, 5.0],
                    [0.0, -20.0, 10.0],
                    [0.0, -25.0, 5.0],
                ]
            ]
        ),
    )

    with patch("general_motion_retargeting.utils.lafan1.read_bvh", return_value=fake_anim), patch(
        "general_motion_retargeting.utils.lafan1.utils.quat_fk",
        return_value=fake_global_data,
    ):
        frames, human_height, metadata = load_bvh_file("fake.bvh", return_metadata=True)

    assert human_height == pytest.approx(1.75)
    assert metadata["joint_names"] == ["Root", "Chest", "LeftFoot", "LeftToe", "RightFoot", "RightToe"]
    np.testing.assert_array_equal(metadata["parents"], np.array([-1, 0, 0, 2, 0, 4]))
    assert "Root" in frames[0]
    assert "Chest" in frames[0]
