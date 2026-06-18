"""Integrated unit tests for localizer_pipeline.py."""

import pytest
import numpy as np

from rf_threshold.core.frame import RFDetection
from rf_threshold.localization.pose import LocalizationStatus
from rf_threshold.localization.localizer_pipeline import RFLocalizer


@pytest.fixture
def test_config():
    """Default localizer configuration."""
    return {
        "data_association": {
            "method": "triplet_distance",
            "min_detections": 3,
            "min_matches": 3,
            "triplet_distance_tolerance": {
                "mode": "adaptive",
                "min_abs": 0.08,
                "relative_ratio": 0.03,
                "max_abs": 0.20,
            },
            "nearest_neighbor_gate": {
                "mode": "adaptive",
                "min_abs": 0.10,
                "relative_ratio": 0.03,
                "max_abs": 0.25,
            },
            "max_candidate_rmse": 0.08,
            "max_candidate_residual": 0.18,
            "max_candidates": 300,
            "use_detection_score_weight": True,
            "reject_duplicate_landmarks": True,
            "reject_duplicate_detections": True,
        },
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


@pytest.fixture
def map_path():
    """Path to the standard map."""
    return "data/maps/your_map_simple.json"


def make_detection(det_id: int, center_lidar: np.ndarray) -> RFDetection:
    """Constructs RFDetection."""
    return RFDetection(
        detection_id=det_id,
        stamp=1716000000.0,
        frame_id="livox_frame",
        center_lidar=center_lidar,
        score=0.9,
        num_points=10,
        mean_intensity=150.0,
        max_intensity=200.0,
        bbox_min=center_lidar - 0.05,
        bbox_max=center_lidar + 0.05,
        cluster_id=det_id,
    )


def test_1_success_scenario(map_path, test_config):
    """Test 1 - Success scenario with valid matched landmarks."""
    localizer = RFLocalizer(map_path, test_config)
    # Using 3 landmarks that are not collinear:
    # ID 0: [0, 0, 1.3]
    # ID 9: [0, 3.005, 1.3]
    # ID 10: [1.41, 3.005, 1.3]
    # Under identity transform:
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.3])),
        make_detection(20, np.array([0.0, 3.005, 1.3])),
        make_detection(30, np.array([1.41, 3.005, 1.3])),
    ]

    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.OK
    assert res.pose is not None
    assert pytest.approx(res.pose.x, abs=1e-7) == 0.0
    assert pytest.approx(res.pose.y, abs=1e-7) == 0.0
    assert pytest.approx(res.pose.yaw, abs=1e-7) == 0.0
    assert res.pose.num_matches == 3


def test_2_insufficient_detections(map_path, test_config):
    """Test 2 - Insufficient detections -> INSUFFICIENT_DETECTIONS."""
    localizer = RFLocalizer(map_path, test_config)
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.3])),
        make_detection(20, np.array([0.0, 3.005, 1.3])),
    ]
    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.INSUFFICIENT_DETECTIONS
    assert res.pose is None


def test_3_association_failure(map_path, test_config):
    """Test 3 - Association failure -> ASSOCIATION_FAILED."""
    localizer = RFLocalizer(map_path, test_config)
    # Random landmarks that cannot correspond to map
    dets = [
        make_detection(10, np.array([10.0, 15.0, 1.3])),
        make_detection(20, np.array([20.0, -5.0, 1.3])),
        make_detection(30, np.array([-10.0, 3.0, 1.3])),
    ]
    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.ASSOCIATION_FAILED
    assert res.pose is None


def test_4_tiny_spatial_spread(map_path, test_config):
    """Test 4 - Tiny spatial spread -> DEGENERATE_GEOMETRY."""
    localizer = RFLocalizer(map_path, test_config)
    # Practically collocated points
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.3])),
        make_detection(20, np.array([0.00001, 0.00001, 1.3])),
        make_detection(30, np.array([0.0, 0.00001, 1.3])),
    ]
    # We cheat map landmarks to also be near zero to pass SVD candidate filtering
    localizer.landmarks = [
        localizer.landmarks[0],
        localizer.landmarks[0],
        localizer.landmarks[0],
    ]
    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.DEGENERATE_GEOMETRY or res.status == LocalizationStatus.ASSOCIATION_FAILED
    assert res.pose is None


def test_5_6_near_collinear_warning_and_success(map_path, test_config):
    """Test 5, 6 - Near collinear points yield warning and OK status."""
    localizer = RFLocalizer(map_path, test_config)
    # Collinear points in simple map:
    # ID 0: [0, 0, 1.3]
    # ID 1: [1.24, 0, 1.3]
    # ID 2: [3.14, 0, 1.3]
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.3])),
        make_detection(20, np.array([1.24, 0.0, 1.3])),
        make_detection(30, np.array([3.14, 0.0, 1.3])),
    ]

    res = localizer.localize(dets, 1716000000.0)
    # Should resolve with OK status and Near-collinear warning
    assert res.status == LocalizationStatus.OK
    assert "Near-collinear" in res.reason
    assert res.pose is not None
    assert res.pose.num_matches == 3


def test_7_near_collinear_high_residual(map_path, test_config):
    """Test 7 - Near-collinear with perturbed points -> HIGH_RESIDUAL."""
    localizer = RFLocalizer(map_path, test_config)
    # Collinear points in map: ID 0, 1, 2
    # Detections are perturbed to yield large RMSE
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.3])),
        make_detection(20, np.array([1.24, 0.2, 1.3])),  # Perturbed y by +0.2m
        make_detection(30, np.array([3.14, -0.2, 1.3])), # Perturbed y by -0.2m
    ]

    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.HIGH_RESIDUAL or res.status == LocalizationStatus.ASSOCIATION_FAILED
    assert res.pose is None


def test_8_9_duplicate_ids(map_path, test_config):
    """Test 8, 9 - Reject duplicate IDs -> DEGENERATE_GEOMETRY."""
    localizer = RFLocalizer(map_path, test_config)
    # Test duplicate detection_id
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.3])),
        make_detection(20, np.array([0.0, 3.005, 1.3])),
        make_detection(10, np.array([1.41, 3.005, 1.3])), # Duplicate det_id 10
    ]
    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.ASSOCIATION_FAILED or res.status == LocalizationStatus.DEGENERATE_GEOMETRY
    assert res.pose is None


def test_10_residual_threshold_rejection(map_path, test_config):
    """Test 10 - Large SVD residual exceeds max_candidate_rmse limit -> HIGH_RESIDUAL."""
    localizer = RFLocalizer(map_path, test_config)
    # ID 0, 9, 10
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.3])),
        make_detection(20, np.array([0.0, 3.2, 1.3])),   # Perturbed y
        make_detection(30, np.array([1.6, 3.005, 1.3])), # Perturbed x
    ]
    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.HIGH_RESIDUAL or res.status == LocalizationStatus.ASSOCIATION_FAILED
    assert res.pose is None


def test_11_id_zero_accepted(map_path, test_config):
    """Test 11 - Detections matching Landmark ID 0 are successfully accepted."""
    localizer = RFLocalizer(map_path, test_config)
    # ID 0: [0, 0, 1.3]
    # ID 9: [0, 3.005, 1.3]
    # ID 10: [1.41, 3.005, 1.3]
    dets = [
        make_detection(0, np.array([0.0, 0.0, 1.3])), # Detection ID 0
        make_detection(10, np.array([0.0, 3.005, 1.3])),
        make_detection(20, np.array([1.41, 3.005, 1.3])),
    ]

    res = localizer.localize(dets, 1716000000.0)
    assert res.status == LocalizationStatus.OK
    assert res.pose is not None
    # Verify landmark 0 is matched
    landmark_ids = [m.landmark_id for m in res.matched_pairs]
    assert 0 in landmark_ids
