"""Unit tests for 2D SVD Pose Solver."""

import pytest
import numpy as np
from rf_threshold.localization.svd_pose import estimate_pose_svd_2d, SVDPoseResult


def test_svd_pose_result_validation():
    """Verify SVDPoseResult post-init validation checks."""
    R = np.eye(2)
    t = np.zeros(2)
    residuals = np.zeros(3)

    # Valid creation
    res = SVDPoseResult(R=R, t=t, yaw=0.0, residuals=residuals, residual_rmse=0.0, num_points=3)
    assert res.num_points == 3

    # Invalid R shape
    with pytest.raises(ValueError):
        SVDPoseResult(R=np.eye(3), t=t, yaw=0.0, residuals=residuals, residual_rmse=0.0, num_points=3)

    # Invalid t shape
    with pytest.raises(ValueError):
        SVDPoseResult(R=R, t=np.zeros(3), yaw=0.0, residuals=residuals, residual_rmse=0.0, num_points=3)

    # Non-finite yaw
    with pytest.raises(ValueError):
        SVDPoseResult(R=R, t=t, yaw=float("nan"), residuals=residuals, residual_rmse=0.0, num_points=3)

    # Invalid residuals dimension
    with pytest.raises(ValueError):
        SVDPoseResult(R=R, t=t, yaw=0.0, residuals=np.zeros((3, 1)), residual_rmse=0.0, num_points=3)

    # Non-finite RMSE
    with pytest.raises(ValueError):
        SVDPoseResult(R=R, t=t, yaw=0.0, residuals=residuals, residual_rmse=float("inf"), num_points=3)

    # Negative num_points
    with pytest.raises(ValueError):
        SVDPoseResult(R=R, t=t, yaw=0.0, residuals=residuals, residual_rmse=0.0, num_points=-1)


def test_1_identity_transform():
    """Test 1 - Identity transform (P_map = P_lidar)."""
    points_lidar = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0],
        [1.0, 1.0]
    ])
    points_map = points_lidar.copy()

    result = estimate_pose_svd_2d(points_lidar, points_map)

    np.testing.assert_allclose(result.R, np.eye(2), atol=1e-7)
    np.testing.assert_allclose(result.t, np.zeros(2), atol=1e-7)
    assert pytest.approx(result.yaw, abs=1e-7) == 0.0
    assert pytest.approx(result.residual_rmse, abs=1e-7) == 0.0
    assert result.num_points == 4


def test_2_translation_only():
    """Test 2 - Translation-only transform."""
    points_lidar = np.array([
        [0.0, 0.0],
        [2.0, 1.0],
        [1.0, 3.0]
    ])
    t_gt = np.array([1.5, -2.3])
    points_map = points_lidar + t_gt

    result = estimate_pose_svd_2d(points_lidar, points_map)

    np.testing.assert_allclose(result.R, np.eye(2), atol=1e-7)
    np.testing.assert_allclose(result.t, t_gt, atol=1e-7)
    assert pytest.approx(result.yaw, abs=1e-7) == 0.0
    assert pytest.approx(result.residual_rmse, abs=1e-7) == 0.0


def test_3_rotation_only():
    """Test 3 - Rotation-only transform."""
    points_lidar = np.array([
        [1.0, 0.0],
        [0.0, 2.0],
        [-1.0, -1.0]
    ])
    # Rotate by 30 degrees (pi/6)
    theta = np.pi / 6.0
    c, s = np.cos(theta), np.sin(theta)
    R_gt = np.array([
        [c, -s],
        [s,  c]
    ])
    points_map = (R_gt @ points_lidar.T).T

    result = estimate_pose_svd_2d(points_lidar, points_map)

    np.testing.assert_allclose(result.R, R_gt, atol=1e-7)
    np.testing.assert_allclose(result.t, np.zeros(2), atol=1e-7)
    assert pytest.approx(result.yaw, abs=1e-7) == theta
    assert pytest.approx(result.residual_rmse, abs=1e-7) == 0.0


def test_4_rotation_and_translation():
    """Test 4 - Rotation and Translation combined."""
    points_lidar = np.array([
        [0.5, 0.5],
        [2.5, 0.5],
        [1.5, 2.5],
        [-0.5, 1.5]
    ])
    # Rotate by -45 degrees (-pi/4) and translate by [1.2, -0.4]
    theta = -np.pi / 4.0
    t_gt = np.array([1.2, -0.4])
    c, s = np.cos(theta), np.sin(theta)
    R_gt = np.array([
        [c, -s],
        [s,  c]
    ])
    points_map = (R_gt @ points_lidar.T).T + t_gt

    result = estimate_pose_svd_2d(points_lidar, points_map)

    np.testing.assert_allclose(result.R, R_gt, atol=1e-7)
    np.testing.assert_allclose(result.t, t_gt, atol=1e-7)
    assert pytest.approx(result.yaw, abs=1e-7) == theta
    assert pytest.approx(result.residual_rmse, abs=1e-7) == 0.0


def test_5_weighted_svd():
    """Test 5 - Weighted SVD solver."""
    points_lidar = np.array([
        [0.0, 0.0],
        [2.0, 0.0],
        [0.0, 2.0]
    ])
    points_map = np.array([
        [0.1, -0.1],  # Perturbed point
        [2.0, 0.0],
        [0.0, 2.0]
    ])
    # Give low weight to the perturbed point
    weights = np.array([0.01, 10.0, 10.0])

    result = estimate_pose_svd_2d(points_lidar, points_map, weights=weights)

    assert np.all(np.isfinite(result.R))
    assert np.all(np.isfinite(result.t))
    assert result.num_points == 3
    assert result.residual_rmse >= 0.0


def test_6_reject_insufficient_points():
    """Test 6 - Reject if less than 3 points are supplied."""
    pts_few = np.array([[0.0, 0.0], [1.0, 1.0]])
    with pytest.raises(ValueError, match="At least 3 points are required"):
        estimate_pose_svd_2d(pts_few, pts_few)


def test_7_reject_mismatched_shapes():
    """Test 7 - Reject if source and target shapes differ."""
    pts_1 = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    pts_2 = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
    with pytest.raises(ValueError, match="shapes must match"):
        estimate_pose_svd_2d(pts_1, pts_2)


def test_8_reject_nan_input():
    """Test 8 - Reject if input contains NaNs/Infs."""
    pts_clean = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    pts_nan = pts_clean.copy()
    pts_nan[0, 0] = float("nan")

    with pytest.raises(ValueError, match="non-finite values"):
        estimate_pose_svd_2d(pts_nan, pts_clean)

    with pytest.raises(ValueError, match="non-finite values"):
        estimate_pose_svd_2d(pts_clean, pts_nan)


def test_9_reject_invalid_weights():
    """Test 9 - Reject various invalid weights cases."""
    pts_1 = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    pts_2 = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])

    # Wrong weights type
    with pytest.raises(TypeError, match="numpy ndarray"):
        estimate_pose_svd_2d(pts_1, pts_2, weights=[1.0, 1.0, 1.0])

    # Incorrect shape
    with pytest.raises(ValueError, match="Weights shape must be"):
        estimate_pose_svd_2d(pts_1, pts_2, weights=np.array([1.0, 1.0]))

    # Contains NaNs
    with pytest.raises(ValueError, match="non-finite values"):
        estimate_pose_svd_2d(pts_1, pts_2, weights=np.array([1.0, float("nan"), 1.0]))

    # Contains zero/negative weights
    with pytest.raises(ValueError, match="strictly positive"):
        estimate_pose_svd_2d(pts_1, pts_2, weights=np.array([1.0, 0.0, 1.0]))

    with pytest.raises(ValueError, match="strictly positive"):
        estimate_pose_svd_2d(pts_1, pts_2, weights=np.array([1.0, -0.5, 1.0]))


def test_10_reject_degenerate_points():
    """Test 10 - Reject if spatial spread is too small."""
    # Collocated points
    pts_collocated = np.array([
        [1.2, 3.4],
        [1.2, 3.4],
        [1.2, 3.4]
    ])
    pts_target = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0]
    ])
    with pytest.raises(ValueError, match="degenerate"):
        estimate_pose_svd_2d(pts_collocated, pts_target)
