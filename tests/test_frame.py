"""Tests for core data structures."""

import numpy as np
import pytest

from rf_threshold.core.frame import LidarFrame, RFCluster, RFDetection


def test_lidar_frame_accepts_valid_shapes():
    points_xyz = np.zeros((5, 3), dtype=float)
    intensity = np.ones(5, dtype=float)

    frame = LidarFrame(
        stamp=1.0,
        frame_id="livox_frame",
        points_xyz=points_xyz,
        intensity=intensity,
    )

    assert frame.points_xyz.shape == (5, 3)
    assert frame.intensity.shape == (5,)


def test_lidar_frame_rejects_mismatched_lengths():
    points_xyz = np.zeros((5, 3), dtype=float)
    intensity = np.ones(4, dtype=float)

    with pytest.raises(ValueError):
        LidarFrame(
            stamp=1.0,
            frame_id="livox_frame",
            points_xyz=points_xyz,
            intensity=intensity,
        )


def test_rf_cluster_accepts_cluster_id_zero():
    cluster = RFCluster(
        cluster_id=0,
        point_indices=np.array([0, 1, 2]),
        points_xyz=np.zeros((3, 3), dtype=float),
        intensity=np.ones(3, dtype=float),
    )

    assert cluster.cluster_id == 0


def test_rf_detection_accepts_detection_id_zero():
    detection = RFDetection(
        detection_id=0,
        stamp=1.0,
        frame_id="livox_frame",
        center_lidar=np.array([1.0, 2.0, 0.0]),
        score=0.8,
        num_points=5,
        mean_intensity=180.0,
        max_intensity=220.0,
        bbox_min=np.array([0.9, 1.9, -0.1]),
        bbox_max=np.array([1.1, 2.1, 0.1]),
        cluster_id=0,
    )

    assert detection.detection_id == 0
    assert detection.cluster_id == 0


def test_rf_detection_rejects_nan_center():
    with pytest.raises(ValueError):
        RFDetection(
            detection_id=0,
            stamp=1.0,
            frame_id="livox_frame",
            center_lidar=np.array([np.nan, 2.0, 0.0]),
            score=0.8,
            num_points=5,
            mean_intensity=180.0,
            max_intensity=220.0,
            bbox_min=np.array([0.9, 1.9, -0.1]),
            bbox_max=np.array([1.1, 2.1, 0.1]),
        )

