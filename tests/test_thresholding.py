"""Tests for the thresholding module."""

import numpy as np
import pytest

from rf_threshold.core.frame import LidarFrame
from rf_threshold.core.thresholding import select_bright_points


def test_select_bright_points_fixed_mode() -> None:
    """Test fixed thresholding mode."""
    points_xyz = np.array(
        [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]], dtype=float
    )
    intensity = np.array([100.0, 150.0, 200.0], dtype=float)

    frame = LidarFrame(
        stamp=1.0,
        frame_id="test_frame",
        points_xyz=points_xyz,
        intensity=intensity,
    )

    cfg = {
        "mode": "fixed",
        "fixed_intensity": 150.0,
    }

    bright_frame, threshold = select_bright_points(frame, cfg)

    assert threshold == 150.0
    assert bright_frame.points_xyz.shape == (2, 3)
    np.testing.assert_array_equal(
        bright_frame.points_xyz, np.array([[2.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    )
    np.testing.assert_array_equal(bright_frame.intensity, np.array([150.0, 200.0]))


def test_select_bright_points_adaptive_mode() -> None:
    """Test adaptive thresholding mode using percentiles."""
    points_xyz = np.array(
        [
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    # Median is 150.0, 75th percentile is 175.0
    intensity = np.array([100.0, 150.0, 200.0, 250.0], dtype=float)

    frame = LidarFrame(
        stamp=1.0,
        frame_id="test_frame",
        points_xyz=points_xyz,
        intensity=intensity,
    )

    cfg = {
        "mode": "adaptive",
        "adaptive": {
            "percentile": 75.0,  # 75th percentile of intensity is 212.5
            "min_intensity": 120.0,
        },
    }

    bright_frame, threshold = select_bright_points(frame, cfg)

    # 75th percentile is 212.5, which is above min_intensity 120.0
    assert threshold == 212.5
    # Only 250.0 is >= 212.5
    assert bright_frame.points_xyz.shape == (1, 3)
    np.testing.assert_array_equal(bright_frame.intensity, np.array([250.0]))


def test_select_bright_points_empty_frame() -> None:
    """Test that select_bright_points handles empty input gracefully."""
    frame = LidarFrame(
        stamp=1.0,
        frame_id="empty",
        points_xyz=np.empty((0, 3), dtype=float),
        intensity=np.empty((0,), dtype=float),
    )
    cfg = {"mode": "fixed", "fixed_intensity": 150.0}

    bright_frame, threshold = select_bright_points(frame, cfg)

    assert threshold == 0.0
    assert bright_frame.points_xyz.shape == (0, 3)
