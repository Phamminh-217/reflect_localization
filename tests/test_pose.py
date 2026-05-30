"""Unit tests for localization data structures."""

import pytest
import numpy as np

from rf_threshold.localization.pose import (
    RFMapLandmark,
    MatchedPair,
    RobotPose,
    LocalizationStatus,
    LocalizationResult,
)


def test_landmark_valid():
    """Test valid RFMapLandmark creation."""
    pos = np.array([1.0, 2.0, 3.0])
    landmark = RFMapLandmark(landmark_id=1, position_map=pos)
    assert landmark.landmark_id == 1
    assert np.array_equal(landmark.position_map, pos)
    assert landmark.frame_id == "map_frame"


def test_landmark_id_zero_valid():
    """Test that landmark_id = 0 is valid."""
    pos = np.array([0.0, 0.0, 0.0])
    landmark = RFMapLandmark(landmark_id=0, position_map=pos)
    assert landmark.landmark_id == 0


def test_landmark_invalid_id():
    """Test that negative landmark_id is rejected."""
    pos = np.array([1.0, 2.0, 3.0])
    with pytest.raises(ValueError, match="landmark_id must be non-negative"):
        RFMapLandmark(landmark_id=-1, position_map=pos)


def test_landmark_invalid_shape():
    """Test that wrong shape for position_map is rejected."""
    pos = np.array([1.0, 2.0])
    with pytest.raises(ValueError, match="position_map must have shape"):
        RFMapLandmark(landmark_id=1, position_map=pos)


def test_landmark_nan():
    """Test that position_map containing NaN is rejected."""
    pos = np.array([1.0, np.nan, 3.0])
    with pytest.raises(ValueError, match="position_map contains NaN or Inf"):
        RFMapLandmark(landmark_id=1, position_map=pos)


def test_matched_pair_valid():
    """Test valid MatchedPair creation."""
    p_lidar = np.array([1.0, 0.5, -0.2])
    p_map = np.array([12.5, 3.2, 1.1])
    match = MatchedPair(
        detection_id=2,
        landmark_id=5,
        point_lidar=p_lidar,
        point_map=p_map,
        weight=2.0,
    )
    assert match.detection_id == 2
    assert match.landmark_id == 5
    assert np.array_equal(match.point_lidar, p_lidar)
    assert np.array_equal(match.point_map, p_map)
    assert match.weight == 2.0


def test_matched_pair_ids_zero():
    """Test that detection_id = 0 and landmark_id = 0 are valid."""
    p_lidar = np.array([0.0, 0.0, 0.0])
    p_map = np.array([0.0, 0.0, 0.0])
    match = MatchedPair(
        detection_id=0,
        landmark_id=0,
        point_lidar=p_lidar,
        point_map=p_map,
    )
    assert match.detection_id == 0
    assert match.landmark_id == 0
    assert match.weight == 1.0


def test_matched_pair_negative_ids():
    """Test that negative IDs are rejected."""
    p = np.array([1.0, 2.0, 3.0])
    with pytest.raises(ValueError, match="detection_id must be non-negative"):
        MatchedPair(detection_id=-1, landmark_id=1, point_lidar=p, point_map=p)
    with pytest.raises(ValueError, match="landmark_id must be non-negative"):
        MatchedPair(detection_id=1, landmark_id=-1, point_lidar=p, point_map=p)


def test_matched_pair_invalid_weight():
    """Test that weight <= 0 is rejected."""
    p = np.array([1.0, 2.0, 3.0])
    with pytest.raises(ValueError, match="weight must be strictly positive"):
        MatchedPair(detection_id=1, landmark_id=1, point_lidar=p, point_map=p, weight=0.0)
    with pytest.raises(ValueError, match="weight must be strictly positive"):
        MatchedPair(detection_id=1, landmark_id=1, point_lidar=p, point_map=p, weight=-1.5)


def test_robot_pose_valid():
    """Test valid RobotPose creation."""
    pose = RobotPose(
        stamp=123.456,
        frame_id="map_frame",
        child_frame_id="lidar_frame",
        x=1.2,
        y=3.4,
        yaw=0.78,
        residual_rmse=0.05,
        num_matches=4,
    )
    assert pose.stamp == 123.456
    assert pose.x == 1.2
    assert pose.y == 3.4
    assert pose.yaw == 0.78
    assert pose.residual_rmse == 0.05
    assert pose.num_matches == 4


def test_robot_pose_nan():
    """Test that RobotPose containing NaN is rejected."""
    with pytest.raises(ValueError, match="x must be finite"):
        RobotPose(
            stamp=123.456,
            frame_id="map",
            child_frame_id="lidar",
            x=np.nan,
            y=3.4,
            yaw=0.78,
            residual_rmse=0.05,
            num_matches=4,
        )


def test_localization_result_ok_valid():
    """Test LocalizationResult status OK with a valid pose."""
    pose = RobotPose(
        stamp=123.456,
        frame_id="map_frame",
        child_frame_id="lidar_frame",
        x=1.2,
        y=3.4,
        yaw=0.78,
        residual_rmse=0.05,
        num_matches=4,
    )
    res = LocalizationResult(
        stamp=123.456,
        status=LocalizationStatus.OK,
        pose=pose,
        matched_pairs=[],
        residual_rmse=0.05,
        reason="Success",
        debug_info={},
    )
    assert res.status == LocalizationStatus.OK
    assert res.pose == pose


def test_localization_result_ok_none_pose():
    """Test that LocalizationResult with status OK but pose=None is rejected."""
    with pytest.raises(ValueError, match="pose cannot be None when status is OK"):
        LocalizationResult(
            stamp=123.456,
            status=LocalizationStatus.OK,
            pose=None,
            matched_pairs=[],
            residual_rmse=None,
            reason="Lacking pose",
            debug_info={},
        )


def test_localization_result_insufficient_matches_valid():
    """Test LocalizationResult status INSUFFICIENT_MATCHES with pose=None is valid."""
    res = LocalizationResult(
        stamp=123.456,
        status=LocalizationStatus.INSUFFICIENT_MATCHES,
        pose=None,
        matched_pairs=[],
        residual_rmse=None,
        reason="No matches found",
        debug_info={},
    )
    assert res.status == LocalizationStatus.INSUFFICIENT_MATCHES
    assert res.pose is None
