"""Unit tests for Triplet Distance-Constrained Data Association."""

import pytest
import numpy as np

from rf_threshold.core.frame import RFDetection
from rf_threshold.localization.pose import RFMapLandmark, LocalizationStatus
from rf_threshold.localization.data_association import (
    associate_detections_to_map,
    compute_adaptive_tolerance,
    TripletDescriptor,
    AssociationCandidate,
    AssociationResult,
)


@pytest.fixture
def base_config():
    """Valid configuration matching standards."""
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
        }
    }


def make_detection(det_id: int, center_lidar: np.ndarray) -> RFDetection:
    """Helper to construct RFDetection."""
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


def make_landmark(landmark_id: int, position_map: np.ndarray) -> RFMapLandmark:
    """Helper to construct RFMapLandmark."""
    return RFMapLandmark(
        landmark_id=landmark_id,
        position_map=position_map,
        frame_id="map_frame",
    )


def test_classes_validations():
    """Verify dataclasses valid structures."""
    desc = TripletDescriptor((0, 1, 2), np.zeros((3, 2)), np.zeros(3))
    assert desc.ids == (0, 1, 2)

    with pytest.raises(ValueError):
        TripletDescriptor((0, 1), np.zeros((3, 2)), np.zeros(3))

    res = AssociationResult(
        status=LocalizationStatus.OK,
        matched_pairs=[],
        residual_rmse=0.01,
        num_inliers=3,
        reason="",
        debug_info={},
    )
    assert res.num_inliers == 3


def test_1_reject_insufficient_detections(base_config):
    """Test 1 - Reject if detections < 3."""
    dets = [
        make_detection(1, np.array([0.0, 0.0, 1.2])),
        make_detection(2, np.array([1.0, 0.0, 1.2])),
    ]
    landmarks = [
        make_landmark(1, np.array([0.0, 0.0, 1.2])),
        make_landmark(2, np.array([1.0, 0.0, 1.2])),
        make_landmark(3, np.array([0.0, 1.0, 1.2])),
    ]
    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.INSUFFICIENT_DETECTIONS
    assert len(res.matched_pairs) == 0


def test_2_reject_insufficient_landmarks(base_config):
    """Test 2 - Reject if landmarks < 3."""
    dets = [
        make_detection(1, np.array([0.0, 0.0, 1.2])),
        make_detection(2, np.array([1.0, 0.0, 1.2])),
        make_detection(3, np.array([0.0, 1.0, 1.2])),
    ]
    landmarks = [
        make_landmark(1, np.array([0.0, 0.0, 1.2])),
        make_landmark(2, np.array([1.0, 0.0, 1.2])),
    ]
    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.MAP_ERROR
    assert len(res.matched_pairs) == 0


def test_3_4_5_adaptive_tolerances():
    """Test 3, 4, 5 - Adaptive tolerance calculation floor, ratio, and cap."""
    # Test 3 - Under floor min_abs
    t1 = compute_adaptive_tolerance(0.5, min_abs=0.08, relative_ratio=0.03, max_abs=0.20)
    assert t1 == 0.08

    # Test 4 - Scaling ratio
    t2 = compute_adaptive_tolerance(4.0, min_abs=0.08, relative_ratio=0.03, max_abs=0.20)
    assert t2 == 0.12  # 4.0 * 0.03 = 0.12

    # Test 5 - Capped by max_abs
    t3 = compute_adaptive_tolerance(10.0, min_abs=0.08, relative_ratio=0.03, max_abs=0.20)
    assert t3 == 0.20


def test_6_match_synthetic_transform(base_config):
    """Test 6 - Match perfect synthetic transform correctly."""
    # Landmarks in map frame
    landmarks = [
        make_landmark(10, np.array([0.0, 0.0, 1.2])),
        make_landmark(20, np.array([2.0, 0.0, 1.2])),
        make_landmark(30, np.array([0.0, 2.0, 1.2])),
    ]

    # Detections rotated by 90 degrees and translated by [1.0, -1.0]
    # R_gt = [[0, -1], [1, 0]], t_gt = [1.0, -1.0]
    # For landmark (0, 0) -> det is (1.0, -1.0)
    # For landmark (2, 0) -> det is (1.0, 1.0)
    # For landmark (0, 2) -> det is (-1.0, -1.0)
    dets = [
        make_detection(1, np.array([1.0, -1.0, 1.2])),
        make_detection(2, np.array([1.0, 1.0, 1.2])),
        make_detection(3, np.array([-1.0, -1.0, 1.2])),
    ]

    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.OK
    assert res.num_inliers == 3
    assert len(res.matched_pairs) == 3

    # Check correspondences
    match_dict = {m.detection_id: m.landmark_id for m in res.matched_pairs}
    assert match_dict[1] == 10
    assert match_dict[2] == 20
    assert match_dict[3] == 30


def test_7_shuffled_order(base_config):
    """Test 7 - Correctly match when detections are shuffled in order."""
    landmarks = [
        make_landmark(1, np.array([0.0, 0.0, 1.2])),
        make_landmark(2, np.array([2.5, 0.0, 1.2])),
        make_landmark(3, np.array([0.0, 2.5, 1.2])),
    ]

    # Shuffled order of landmarks under identity transform
    dets = [
        make_detection(10, np.array([2.5, 0.0, 1.2])),  # maps to 2
        make_detection(20, np.array([0.0, 2.5, 1.2])),  # maps to 3
        make_detection(30, np.array([0.0, 0.0, 1.2])),  # maps to 1
    ]

    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.OK
    match_dict = {m.detection_id: m.landmark_id for m in res.matched_pairs}
    assert match_dict[10] == 2
    assert match_dict[20] == 3
    assert match_dict[30] == 1


def test_8_noisy_spurious_detections(base_config):
    """Test 8 - Correctly matches when spurious noise detections are present."""
    landmarks = [
        make_landmark(1, np.array([0.0, 0.0, 1.2])),
        make_landmark(2, np.array([3.0, 0.0, 1.2])),
        make_landmark(3, np.array([0.0, 3.0, 1.2])),
    ]

    dets = [
        make_detection(100, np.array([10.0, 10.0, 1.2])),  # Noise outlier
        make_detection(10, np.array([3.0, 0.0, 1.2])),    # maps to 2
        make_detection(20, np.array([0.0, 3.0, 1.2])),    # maps to 3
        make_detection(30, np.array([0.0, 0.0, 1.2])),    # maps to 1
    ]

    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.OK
    assert res.num_inliers == 3
    match_dict = {m.detection_id: m.landmark_id for m in res.matched_pairs}
    assert 100 not in match_dict
    assert match_dict[10] == 2
    assert match_dict[20] == 3
    assert match_dict[30] == 1


def test_9_reject_by_rmse(base_config):
    """Test 9 - Reject if final RMSE exceeds max_candidate_rmse threshold."""
    landmarks = [
        make_landmark(1, np.array([0.0, 0.0, 1.2])),
        make_landmark(2, np.array([2.0, 0.0, 1.2])),
        make_landmark(3, np.array([0.0, 2.0, 1.2])),
    ]

    # Large perturbation causing high SVD residual
    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.2])),
        make_detection(20, np.array([2.3, 0.0, 1.2])),  # Perturbed +0.3m
        make_detection(30, np.array([0.0, 2.3, 1.2])),  # Perturbed +0.3m
    ]

    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.ASSOCIATION_FAILED
    assert len(res.matched_pairs) == 0


def test_10_11_reject_duplicates(base_config):
    """Test 10, 11 - One-to-one mapping enforcement prevents duplicates."""
    # Under greedy one-to-one bipartite matching:
    # 1. A detection must map to at most 1 landmark
    # 2. A landmark must map to at most 1 detection
    landmarks = [
        make_landmark(1, np.array([0.0, 0.0, 1.2])),
        make_landmark(2, np.array([2.0, 0.0, 1.2])),
        make_landmark(3, np.array([0.0, 2.0, 1.2])),
    ]

    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.2])),
        make_detection(20, np.array([2.0, 0.0, 1.2])),
        # Det 30 is placed right next to Det 20 to tempt double matching landmark 2
        make_detection(30, np.array([2.01, 0.01, 1.2])),
    ]

    res = associate_detections_to_map(dets, landmarks, base_config)
    # One-to-one constraint should cleanly resolve it without double mappings
    assert res.status == LocalizationStatus.ASSOCIATION_FAILED or res.status == LocalizationStatus.OK
    if res.status == LocalizationStatus.OK:
        landmark_ids = [m.landmark_id for m in res.matched_pairs]
        detection_ids = [m.detection_id for m in res.matched_pairs]
        # Check no duplicates
        assert len(landmark_ids) == len(set(landmark_ids))
        assert len(detection_ids) == len(set(detection_ids))


def test_12_id_zero_is_valid(base_config):
    """Test 12 - Work properly when detection_id or landmark_id are 0."""
    landmarks = [
        make_landmark(0, np.array([0.0, 0.0, 1.2])),  # Landmark ID 0
        make_landmark(1, np.array([2.0, 0.0, 1.2])),
        make_landmark(2, np.array([0.0, 2.0, 1.2])),
    ]

    dets = [
        make_detection(0, np.array([0.0, 0.0, 1.2])),  # Detection ID 0
        make_detection(10, np.array([2.0, 0.0, 1.2])),
        make_detection(20, np.array([0.0, 2.0, 1.2])),
    ]

    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.OK
    match_dict = {m.detection_id: m.landmark_id for m in res.matched_pairs}
    assert match_dict[0] == 0


def test_13_no_direct_nearest_neighbor(base_config):
    """Test 13 - Ensure no direct nearest neighbor is used without transform."""
    # Landmarks are far from detections (no spatial overlap in raw coordinates)
    landmarks = [
        make_landmark(1, np.array([100.0, 100.0, 1.2])),
        make_landmark(2, np.array([102.0, 100.0, 1.2])),
        make_landmark(3, np.array([100.0, 102.0, 1.2])),
    ]

    dets = [
        make_detection(10, np.array([0.0, 0.0, 1.2])),
        make_detection(20, np.array([2.0, 0.0, 1.2])),
        make_detection(30, np.array([0.0, 2.0, 1.2])),
    ]

    # Without triplet-based SVD first, raw coordinates distance is ~141m.
    # Triplet association finds the transform perfectly and aligns them.
    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.OK
    assert res.num_inliers == 3


def test_14_graceful_failures(base_config):
    """Test 14 - Gracefully returns ASSOCIATION_FAILED on complete noise."""
    landmarks = [
        make_landmark(1, np.array([0.0, 0.0, 1.2])),
        make_landmark(2, np.array([2.0, 0.0, 1.2])),
        make_landmark(3, np.array([0.0, 2.0, 1.2])),
    ]

    # Random points that cannot form a rigid transformation
    dets = [
        make_detection(10, np.array([5.0, 9.0, 1.2])),
        make_detection(20, np.array([-1.0, 4.0, 1.2])),
        make_detection(30, np.array([12.0, -3.0, 1.2])),
    ]

    res = associate_detections_to_map(dets, landmarks, base_config)
    assert res.status == LocalizationStatus.ASSOCIATION_FAILED
    assert len(res.matched_pairs) == 0
