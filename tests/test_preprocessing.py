"""Tests for the preprocessing module."""

import numpy as np
import pytest

from rf_threshold.core.frame import LidarFrame
from rf_threshold.core.preprocessing import preprocess_frame, remove_invalid_points


def test_remove_invalid_points_filters_nan_and_inf() -> None:
    """Test that NaN and Inf coordinates and intensities are correctly removed."""
    points_xyz = np.array(
        [
            [1.0, 2.0, 3.0],
            [np.nan, 2.0, 3.0],  # NaN x
            [1.0, np.inf, 3.0],  # Inf y
            [4.0, 5.0, 6.0],
            [7.0, 8.0, np.nan],  # NaN z
        ],
        dtype=float,
    )
    intensity = np.array([100.0, 150.0, 200.0, 250.0, np.nan], dtype=float)  # NaN intensity

    frame = LidarFrame(
        stamp=1.0,
        frame_id="test_frame",
        points_xyz=points_xyz,
        intensity=intensity,
    )

    filtered = remove_invalid_points(frame)

    # Only point 0 (1, 2, 3) and point 3 (4, 5, 6) are completely finite
    assert filtered.points_xyz.shape == (2, 3)
    assert filtered.intensity.shape == (2,)
    np.testing.assert_array_equal(
        filtered.points_xyz, np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    )
    np.testing.assert_array_equal(filtered.intensity, np.array([100.0, 250.0]))


def test_preprocess_frame_range_filter() -> None:
    """Test range filter in preprocessing."""
    points_xyz = np.array(
        [
            [0.1, 0.0, 0.0],  # range = 0.1 (too close)
            [1.0, 0.0, 0.0],  # range = 1.0 (valid)
            [5.0, 0.0, 0.0],  # range = 5.0 (valid)
            [9.0, 0.0, 0.0],  # range = 9.0 (too far)
        ],
        dtype=float,
    )
    intensity = np.array([100.0, 150.0, 200.0, 250.0], dtype=float)

    frame = LidarFrame(
        stamp=1.0,
        frame_id="test_frame",
        points_xyz=points_xyz,
        intensity=intensity,
    )

    cfg = {
        "remove_nan": True,
        "range_filter": {
            "enabled": True,
            "min_range": 0.2,
            "max_range": 8.0,
        },
        "height_filter": {
            "enabled": False,
        },
    }

    filtered, summary = preprocess_frame(frame, cfg)

    assert filtered.points_xyz.shape == (2, 3)
    np.testing.assert_array_equal(
        filtered.points_xyz, np.array([[1.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
    )

    assert summary["raw_points"] == 4
    assert summary["valid_points"] == 4
    assert summary["range_filtered_points"] == 2
    assert summary["height_filtered_points"] == 2  # height filter disabled, count remains same


def test_preprocess_frame_height_filter() -> None:
    """Test height filter in preprocessing."""
    points_xyz = np.array(
        [
            [1.0, 1.0, -0.6],  # too low
            [1.0, 1.0, 0.0],  # valid
            [1.0, 1.0, 0.4],  # valid
            [1.0, 1.0, 0.6],  # too high
        ],
        dtype=float,
    )
    intensity = np.array([100.0, 150.0, 200.0, 250.0], dtype=float)

    frame = LidarFrame(
        stamp=1.0,
        frame_id="test_frame",
        points_xyz=points_xyz,
        intensity=intensity,
    )

    cfg = {
        "remove_nan": True,
        "range_filter": {
            "enabled": False,
        },
        "height_filter": {
            "enabled": True,
            "min_z": -0.5,
            "max_z": 0.5,
        },
    }

    filtered, summary = preprocess_frame(frame, cfg)

    assert filtered.points_xyz.shape == (2, 3)
    np.testing.assert_array_equal(
        filtered.points_xyz, np.array([[1.0, 1.0, 0.0], [1.0, 1.0, 0.4]])
    )

    assert summary["raw_points"] == 4
    assert summary["valid_points"] == 4
    assert summary["range_filtered_points"] == 4
    assert summary["height_filtered_points"] == 2
