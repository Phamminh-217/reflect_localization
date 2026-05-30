"""Tests for the clustering module."""

import numpy as np
import pytest

from rf_threshold.core.clustering import cluster_bright_points
from rf_threshold.core.frame import LidarFrame


def test_cluster_bright_points_success() -> None:
    """Test that points are grouped into appropriate clusters by DBSCAN."""
    # Create two clear clusters separated in space
    points_xyz = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.01, 0.01, 0.0],
            [0.02, 0.0, 0.0],  # Cluster 1
            [5.0, 5.0, 0.0],
            [5.01, 5.01, 0.0],
            [5.02, 5.0, 0.0],  # Cluster 2
        ],
        dtype=float,
    )
    intensity = np.array([200.0, 210.0, 220.0, 180.0, 190.0, 200.0], dtype=float)

    frame = LidarFrame(
        stamp=1.0,
        frame_id="test",
        points_xyz=points_xyz,
        intensity=intensity,
    )

    cfg = {
        "method": "dbscan",
        "eps": 0.1,
        "min_samples": 2,
        "use_dimension": "xy",
    }

    clusters = cluster_bright_points(frame, cfg)

    assert len(clusters) == 2
    # Verify cluster sizes
    sizes = sorted([c.points_xyz.shape[0] for c in clusters])
    assert sizes == [3, 3]


def test_cluster_bright_points_empty_input() -> None:
    """Test that DBSCAN handles empty input gracefully without crash."""
    frame = LidarFrame(
        stamp=1.0,
        frame_id="empty",
        points_xyz=np.empty((0, 3), dtype=float),
        intensity=np.empty((0,), dtype=float),
    )
    cfg = {
        "method": "dbscan",
        "eps": 0.1,
        "min_samples": 2,
        "use_dimension": "xy",
    }

    clusters = cluster_bright_points(frame, cfg)

    assert clusters == []
