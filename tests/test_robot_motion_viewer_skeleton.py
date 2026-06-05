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


def test_build_skeleton_bone_segments_can_connect_to_nearest_available_ancestor():
    motion_data = {
        "Hips": (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
        "Spine2": (np.array([0.0, 0.0, 1.5]), np.array([1.0, 0.0, 0.0, 0.0])),
        "LeftArm": (np.array([0.3, 0.0, 1.45]), np.array([1.0, 0.0, 0.0, 0.0])),
    }

    segments = build_skeleton_bone_segments(
        motion_data,
        joint_names=["Hips", "Spine", "Spine1", "Spine2", "LeftShoulder", "LeftArm"],
        parents=np.array([-1, 0, 1, 2, 3, 4]),
        connect_to_nearest_available=True,
    )

    assert [(parent_name, child_name) for parent_name, child_name, _, _ in segments] == [
        ("Hips", "Spine2"),
        ("Spine2", "LeftArm"),
    ]


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
from scripts.bvh_to_robot import make_human_skeleton_payloads


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
    np.testing.assert_allclose(payloads[0]["pos_offset"], np.array([0.0, 0.0, 0.2]))
    assert "pos_offset" not in payloads[1]


def test_make_human_skeleton_payloads_aligns_raw_root_to_scaled_root():
    raw_frame = {
        "Hips": (np.array([1.0, 2.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
    }
    scaled_frame = {
        "Hips": (np.array([1.3, 2.5, 1.2]), np.array([1.0, 0.0, 0.0, 0.0])),
    }
    metadata = {"joint_names": ["Hips"], "parents": np.array([-1])}

    payloads = make_human_skeleton_payloads(raw_frame, scaled_frame, metadata)

    np.testing.assert_allclose(payloads[0]["pos_offset"], np.array([0.3, 0.5, 0.2]))
    assert "pos_offset" not in payloads[1]


def test_make_human_skeleton_payloads_adds_scaled_helper_feet_and_compressed_edges():
    raw_frame = {
        "Hips": (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
    }
    scaled_frame = {
        "Hips": (np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0])),
        "LeftLeg": (np.array([0.1, 0.0, 0.5]), np.array([1.0, 0.0, 0.0, 0.0])),
        "LeftFootMod": (np.array([0.2, 0.0, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
    }
    metadata = {
        "joint_names": ["Hips", "LeftUpLeg", "LeftLeg", "LeftFoot"],
        "parents": np.array([-1, 0, 1, 2]),
    }

    payloads = make_human_skeleton_payloads(raw_frame, scaled_frame, metadata)
    scaled_payload = payloads[1]

    assert scaled_payload["connect_to_nearest_available"] is True
    assert scaled_payload["joint_names"] == ["Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftFootMod"]
    np.testing.assert_array_equal(scaled_payload["parents"], np.array([-1, 0, 1, 2, 2]))
