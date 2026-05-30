"""Tests for the center_estimation module."""

import numpy as np
import pytest

from rf_threshold.core.center_estimation import (
    estimate_centroid,
    estimate_intensity_weighted_center,
)
from rf_threshold.core.frame import RFCluster


def test_estimate_centroid() -> None:
    """Test simple centroid calculation."""
    points_xyz = np.array(
        [
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ],
        dtype=float,
    )
    center = estimate_centroid(points_xyz)
    np.testing.assert_allclose(center, np.array([4.0, 5.0, 6.0]))


def test_estimate_centroid_empty() -> None:
    """Test that empty point cloud returns zero coordinates."""
    points_xyz = np.empty((0, 3), dtype=float)
    center = estimate_centroid(points_xyz)
    np.testing.assert_allclose(center, np.zeros(3))


def test_estimate_intensity_weighted_center_success() -> None:
    """Test successful intensity weighted center calculation."""
    cluster = RFCluster(
        cluster_id=1,
        point_indices=np.array([0, 1, 2]),
        points_xyz=np.array(
            [
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ],
            dtype=float,
        ),
        # Weighted heavily on point 2
        intensity=np.array([150.0, 150.0, 250.0], dtype=float),
    )

    cfg = {
        "intensity_weight_power": 1.0,
        "clamp_percentile": 100.0,  # no clamping
    }

    center = estimate_intensity_weighted_center(cluster, 140.0, cfg)

    # weights: w = max(I - 140, 0) => w = [10.0, 10.0, 110.0]
    # sum(w) = 130.0
    # c_x = (10*1 + 10*2 + 110*3) / 130 = (10 + 20 + 330) / 130 = 360 / 130 = 2.769
    np.testing.assert_allclose(center[0], 360.0 / 130.0, rtol=1e-5)
    assert center[1] == 0.0
    assert center[2] == 0.0


def test_estimate_intensity_weighted_center_fallback() -> None:
    """Test fallback to simple centroid when total weight sum is zero."""
    cluster = RFCluster(
        cluster_id=1,
        point_indices=np.array([0, 1, 2]),
        points_xyz=np.array(
            [
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ],
            dtype=float,
        ),
        # Intensities all below threshold, weights will be zero
        intensity=np.array([100.0, 110.0, 120.0], dtype=float),
    )

    cfg = {
        "intensity_weight_power": 1.0,
        "clamp_percentile": 95.0,
    }

    # threshold = 130.0 (all intensities are below this)
    center = estimate_intensity_weighted_center(cluster, 130.0, cfg)

    # Fallback to mean points: mean(1, 2, 3) = 2.0
    np.testing.assert_allclose(center, np.array([2.0, 0.0, 0.0]))
