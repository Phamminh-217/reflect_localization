"""Tests for the cluster_validation module."""

import numpy as np
import pytest

from rf_threshold.core.cluster_validation import validate_cluster_with_reason
from rf_threshold.core.frame import RFCluster


def test_validate_cluster_success() -> None:
    """Test successful validation of a valid cluster."""
    cluster = RFCluster(
        cluster_id=1,
        point_indices=np.array([0, 1, 2, 3]),
        points_xyz=np.array(
            [
                [0.0, 0.0, 0.0],
                [0.05, 0.05, 0.1],
                [-0.05, -0.05, -0.1],
                [0.0, 0.0, 0.0],
            ],
            dtype=float,
        ),  # extents: x=0.1, y=0.1, z=0.2
        intensity=np.array([200.0, 180.0, 160.0, 190.0], dtype=float),  # mean = 182.5
    )

    cfg = {
        "min_points": 3,
        "max_points": 10,
        "max_extent_x": 0.30,
        "max_extent_y": 0.30,
        "max_extent_z": 0.50,
        "min_mean_intensity": 120.0,
    }

    valid, reason = validate_cluster_with_reason(cluster, cfg)

    assert valid is True
    assert reason == "valid"


def test_validate_cluster_too_few_points() -> None:
    """Test validation failure due to insufficient points."""
    cluster = RFCluster(
        cluster_id=1,
        point_indices=np.array([0, 1]),
        points_xyz=np.zeros((2, 3), dtype=float),
        intensity=np.array([200.0, 200.0], dtype=float),
    )

    cfg = {
        "min_points": 3,
        "max_points": 10,
        "max_extent_x": 0.30,
        "max_extent_y": 0.30,
        "max_extent_z": 0.50,
        "min_mean_intensity": 120.0,
    }

    valid, reason = validate_cluster_with_reason(cluster, cfg)

    assert valid is False
    assert reason == "num_points_too_small"


def test_validate_cluster_extent_too_large() -> None:
    """Test validation failure due to excessive bounding box size."""
    cluster = RFCluster(
        cluster_id=1,
        point_indices=np.array([0, 1, 2]),
        points_xyz=np.array(
            [[0.0, 0.0, 0.0], [0.4, 0.0, 0.0], [0.2, 0.0, 0.0]], dtype=float
        ),  # extent x = 0.4 (too large)
        intensity=np.array([200.0, 200.0, 200.0], dtype=float),
    )

    cfg = {
        "min_points": 3,
        "max_points": 10,
        "max_extent_x": 0.30,
        "max_extent_y": 0.30,
        "max_extent_z": 0.50,
        "min_mean_intensity": 120.0,
    }

    valid, reason = validate_cluster_with_reason(cluster, cfg)

    assert valid is False
    assert reason == "extent_x_too_large"


def test_validate_cluster_intensity_too_low() -> None:
    """Test validation failure due to low mean intensity."""
    cluster = RFCluster(
        cluster_id=1,
        point_indices=np.array([0, 1, 2]),
        points_xyz=np.zeros((3, 3), dtype=float),
        intensity=np.array([100.0, 110.0, 120.0], dtype=float),  # mean = 110.0 (too low)
    )

    cfg = {
        "min_points": 3,
        "max_points": 10,
        "max_extent_x": 0.30,
        "max_extent_y": 0.30,
        "max_extent_z": 0.50,
        "min_mean_intensity": 150.0,
    }

    valid, reason = validate_cluster_with_reason(cluster, cfg)

    assert valid is False
    assert reason == "mean_intensity_too_low"
