"""Unit tests for geometry_check.py."""

import pytest
import numpy as np

from rf_threshold.localization.pose import MatchedPair
from rf_threshold.localization.geometry_check import check_geometry_validity, GeometryCheckResult


@pytest.fixture
def base_config():
    """Default geometry check config."""
    return {
        "geometry_check": {
            "min_matches": 3,
            "min_spread": 0.30,
            "condition_number": {
                "enabled": True,
                "max_condition_number": 50.0,
                "hard_reject": False,
            }
        }
    }


def make_pair(det_id: int, l_id: int, pt_lidar: np.ndarray, pt_map: np.ndarray) -> MatchedPair:
    """Helper to create MatchedPair."""
    return MatchedPair(
        detection_id=det_id,
        landmark_id=l_id,
        point_lidar=pt_lidar,
        point_map=pt_map,
    )


def test_geometry_check_result_typing():
    """Verify GeometryCheckResult validation."""
    res = GeometryCheckResult(
        is_valid=True,
        is_degenerate=False,
        warning=None,
        reason="",
        num_pairs=3,
        condition_number_lidar=1.0,
        condition_number_map=1.0,
        spread_lidar=1.0,
        spread_map=1.0,
        debug_info={},
    )
    assert res.is_valid

    with pytest.raises(TypeError):
        # Invalid is_valid type
        GeometryCheckResult(
            is_valid="yes",
            is_degenerate=False,
            warning=None,
            reason="",
            num_pairs=3,
            condition_number_lidar=1.0,
            condition_number_map=1.0,
            spread_lidar=1.0,
            spread_map=1.0,
            debug_info={},
        )


def test_insufficient_points(base_config):
    """Test - Reject if matched pairs < min_matches."""
    pairs = [
        make_pair(1, 1, np.array([0.0, 0.0, 1.2]), np.array([0.0, 0.0, 1.2])),
        make_pair(2, 2, np.array([1.0, 0.0, 1.2]), np.array([1.0, 0.0, 1.2])),
    ]
    res = check_geometry_validity(pairs, base_config)
    assert not res.is_valid
    assert res.warning == "INSUFFICIENT_POINTS"


def test_duplicate_detection_ids(base_config):
    """Test - Reject if duplicate detection_id exists."""
    pairs = [
        make_pair(1, 1, np.array([0.0, 0.0, 1.2]), np.array([0.0, 0.0, 1.2])),
        make_pair(2, 2, np.array([1.0, 0.0, 1.2]), np.array([1.0, 0.0, 1.2])),
        make_pair(1, 3, np.array([0.0, 1.0, 1.2]), np.array([0.0, 1.0, 1.2])), # Duplicate det_id 1
    ]
    res = check_geometry_validity(pairs, base_config)
    assert not res.is_valid
    assert res.warning == "DUPLICATE_IDS"


def test_duplicate_landmark_ids(base_config):
    """Test - Reject if duplicate landmark_id exists."""
    pairs = [
        make_pair(1, 1, np.array([0.0, 0.0, 1.2]), np.array([0.0, 0.0, 1.2])),
        make_pair(2, 2, np.array([1.0, 0.0, 1.2]), np.array([1.0, 0.0, 1.2])),
        make_pair(3, 1, np.array([0.0, 1.0, 1.2]), np.array([0.0, 1.0, 1.2])), # Duplicate landmark_id 1
    ]
    res = check_geometry_validity(pairs, base_config)
    assert not res.is_valid
    assert res.warning == "DUPLICATE_IDS"


def test_nan_coordinates(base_config):
    """Test - Reject if point coordinates contain NaN."""
    p3 = make_pair(3, 3, np.array([0.0, 1.0, 1.2]), np.array([0.0, 1.0, 1.2]))
    p3.point_lidar[1] = float("nan")
    pairs = [
        make_pair(1, 1, np.array([0.0, 0.0, 1.2]), np.array([0.0, 0.0, 1.2])),
        make_pair(2, 2, np.array([1.0, 0.0, 1.2]), np.array([1.0, 0.0, 1.2])),
        p3,
    ]
    res = check_geometry_validity(pairs, base_config)
    assert not res.is_valid
    assert res.warning == "INVALID_COORDINATES"


def test_1_tiny_spread_rejected(base_config):
    """Test 1 - Reject if points are nearly identical (tiny spatial spread)."""
    # Points are virtually collocated (spread = 0.0)
    pairs = [
        make_pair(1, 1, np.array([1.2, 3.4, 1.2]), np.array([1.2, 3.4, 1.2])),
        make_pair(2, 2, np.array([1.2, 3.4, 1.2]), np.array([1.2, 3.4, 1.2])),
        make_pair(3, 3, np.array([1.2, 3.4, 1.2]), np.array([1.2, 3.4, 1.2])),
    ]
    res = check_geometry_validity(pairs, base_config)
    assert not res.is_valid
    assert res.is_degenerate
    assert res.warning == "DEGENERATE_GEOMETRY"


def test_2_near_collinear_warning_only(base_config):
    """Test 2 - Allow near-collinear points with a warning by default (hard_reject = False)."""
    # Points lay almost perfectly on a straight line y = 0
    # spread is large (2.0) but S2 is very small -> high condition number
    pairs = [
        make_pair(1, 1, np.array([0.0, 0.0, 1.2]), np.array([0.0, 0.0, 1.2])),
        make_pair(2, 2, np.array([1.0, 1e-5, 1.2]), np.array([1.0, 1e-5, 1.2])),
        make_pair(3, 3, np.array([2.0, 0.0, 1.2]), np.array([2.0, 0.0, 1.2])),
    ]
    res = check_geometry_validity(pairs, base_config)
    assert res.is_valid
    assert res.is_degenerate
    assert res.warning == "NEAR_COLLINEAR"
    assert res.condition_number_lidar > 50.0


def test_collinear_hard_reject():
    """Test - Reject near-collinear points if config explicitly enables hard_reject."""
    cfg = {
        "geometry_check": {
            "min_matches": 3,
            "min_spread": 0.30,
            "condition_number": {
                "enabled": True,
                "max_condition_number": 50.0,
                "hard_reject": True,  # Kích hoạt từ chối cứng
            }
        }
    }
    pairs = [
        make_pair(1, 1, np.array([0.0, 0.0, 1.2]), np.array([0.0, 0.0, 1.2])),
        make_pair(2, 2, np.array([1.0, 1e-5, 1.2]), np.array([1.0, 1e-5, 1.2])),
        make_pair(3, 3, np.array([2.0, 0.0, 1.2]), np.array([2.0, 0.0, 1.2])),
    ]
    res = check_geometry_validity(pairs, cfg)
    assert not res.is_valid
    assert res.is_degenerate
    assert res.warning == "NEAR_COLLINEAR"
    assert "exceeds threshold" in res.reason
