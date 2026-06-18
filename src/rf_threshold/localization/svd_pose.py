"""Module for 2D SVD-based rigid pose estimation (registration)."""

from dataclasses import dataclass
import logging
from typing import Optional
import numpy as np

logger = logging.getLogger("svd_pose")


@dataclass(frozen=True)
class SVDPoseResult:
    """Represents the output of the 2D SVD pose estimation.

    Args:
        R: 2x2 rotation matrix.
        t: 2D translation vector [x, y].
        yaw: Yaw angle in radians.
        residuals: Array of shape (N,) containing residual distance for each point.
        residual_rmse: Root Mean Square Error of the registration.
        num_points: Number of point pairs used in the solution.
    """
    R: np.ndarray
    t: np.ndarray
    yaw: float
    residuals: np.ndarray
    residual_rmse: float
    num_points: int

    def __post_init__(self) -> None:
        """Validate result structure."""
        if self.R.shape != (2, 2):
            raise ValueError(f"R must have shape (2, 2), got {self.R.shape}")
        if self.t.shape != (2,):
            raise ValueError(f"t must have shape (2,), got {self.t.shape}")
        if not np.isfinite(self.yaw):
            raise ValueError("yaw must be finite")
        if self.residuals.ndim != 1:
            raise ValueError(f"residuals must be 1D, got {self.residuals.ndim}D")
        if not np.isfinite(self.residual_rmse):
            raise ValueError("residual_rmse must be finite")
        if self.num_points < 0:
            raise ValueError("num_points must be non-negative")


def check_points_2d_valid_for_svd(points_xy: np.ndarray) -> None:
    """Check if the given 2D points are geometrically valid for SVD registration.

    Args:
        points_xy: NumPy array of shape (N, 2).

    Raises:
        ValueError: If points contain NaNs/Infs, have less than 3 points, or spatial spread is too small.
    """
    if points_xy.ndim != 2 or points_xy.shape[1] != 2:
        raise ValueError(f"Points must have shape (N, 2), got {points_xy.shape}")

    n_points = points_xy.shape[0]
    if n_points < 3:
        raise ValueError(f"At least 3 points are required for SVD, got {n_points}")

    if not np.all(np.isfinite(points_xy)):
        raise ValueError("Points contain non-finite values (NaN or Inf)")

    # Spatial spread check
    max_coords = np.max(points_xy, axis=0)
    min_coords = np.min(points_xy, axis=0)
    spread = float(np.linalg.norm(max_coords - min_coords))
    if spread < 1e-6:
        logger.error("Spatial spread is too small: %f", spread)
        raise ValueError(f"Points are degenerate: spatial spread {spread:.2e} is below threshold 1e-6")


def estimate_pose_svd_2d(
    points_lidar_xy: np.ndarray,
    points_map_xy: np.ndarray,
    weights: Optional[np.ndarray] = None,
) -> SVDPoseResult:
    """Estimate a 2D rigid transform T_map_lidar = (R, t) using Singular Value Decomposition.

    The transform maps points from the lidar frame to the map frame:
        point_map ≈ R * point_lidar + t

    Args:
        points_lidar_xy: Shape (N, 2), source coordinates in lidar frame.
        points_map_xy: Shape (N, 2), target coordinates in map frame.
        weights: Optional shape (N,), positive weight for each matched pair.

    Returns:
        An SVDPoseResult containing the optimal rotation R, translation t, yaw, and residuals.

    Raises:
        ValueError: If input shapes, counts, finite checks, weights, or degeneracy checks fail.
    """
    # 1. Base input validations
    if not isinstance(points_lidar_xy, np.ndarray) or not isinstance(points_map_xy, np.ndarray):
        raise TypeError("Inputs must be numpy ndarrays")

    if points_lidar_xy.shape != points_map_xy.shape:
        raise ValueError(f"Source and target shapes must match, got {points_lidar_xy.shape} and {points_map_xy.shape}")

    # 2. Check geometry and finite values
    check_points_2d_valid_for_svd(points_lidar_xy)
    check_points_2d_valid_for_svd(points_map_xy)

    n_points = points_lidar_xy.shape[0]

    # 3. Weights validation
    if weights is not None:
        if not isinstance(weights, np.ndarray):
            raise TypeError("Weights must be a numpy ndarray")
        if weights.shape != (n_points,):
            raise ValueError(f"Weights shape must be ({n_points},), got {weights.shape}")
        if not np.all(np.isfinite(weights)):
            raise ValueError("Weights contain non-finite values (NaN or Inf)")
        if np.any(weights <= 0.0):
            raise ValueError("All weights must be strictly positive (> 0.0)")
        
        sum_w = np.sum(weights)
        if sum_w <= 0.0:
            raise ValueError(f"Sum of weights must be positive, got {sum_w}")

        # Normalize weights so they sum to 1
        w_norm = weights / sum_w
    else:
        # Uniform weights
        w_norm = np.ones(n_points, dtype=np.float64) / n_points

    # 4. Calculate weighted centroids
    mu_lidar = np.sum(points_lidar_xy * w_norm[:, None], axis=0)
    mu_map = np.sum(points_map_xy * w_norm[:, None], axis=0)

    # 5. Centered coordinates
    lidar_centered = points_lidar_xy - mu_lidar
    map_centered = points_map_xy - mu_map

    # 6. Covariance matrix H
    # H = sum(w_i * lidar_centered_i^T * map_centered_i)
    H = (lidar_centered * w_norm[:, None]).T @ map_centered

    # 7. SVD of H
    U, _, Vt = np.linalg.svd(H)

    # 8. Compute R = V @ U^T
    # Vt in numpy SVD is actually V^T, so V = Vt.T
    R = Vt.T @ U.T

    # 9. Correct reflection if det(R) < 0
    det_R = np.linalg.det(R)
    if det_R < 0:
        logger.debug("Reflection detected (det(R) = %f). Correcting...", det_R)
        # Create reflection correction matrix
        correction = np.diag([1.0, -1.0])
        R = Vt.T @ correction @ U.T

    # 10. Compute t = mu_map - R @ mu_lidar
    t = mu_map - R @ mu_lidar

    # 11. Compute yaw
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))

    # 12. Calculate residuals and RMSE
    predicted_map_xy = (R @ points_lidar_xy.T).T + t
    residuals = np.linalg.norm(points_map_xy - predicted_map_xy, axis=1)
    
    # Weighted Root Mean Square Error
    residual_rmse = float(np.sqrt(np.sum(w_norm * (residuals ** 2))))

    return SVDPoseResult(
        R=R,
        t=t,
        yaw=yaw,
        residuals=residuals,
        residual_rmse=residual_rmse,
        num_points=n_points,
    )
