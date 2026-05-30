"""Module for estimating the physical centers of RF landmark clusters."""

import logging
from typing import Any, Dict

import numpy as np

from rf_threshold.core.frame import RFCluster

logger = logging.getLogger("center_estimation")

EPSILON = 1e-8


def estimate_centroid(points_xyz: np.ndarray) -> np.ndarray:
    """Compute the simple arithmetic mean (centroid) of a point cloud.

    Args:
        points_xyz: Coordinate array with shape (N, 3).

    Returns:
        The calculated center coordinates as shape (3,).
    """
    if points_xyz.shape[0] == 0:
        return np.zeros(3, dtype=np.float64)

    return np.mean(points_xyz, axis=0)


def estimate_intensity_weighted_center(
    cluster: RFCluster,
    threshold: float,
    cfg: Dict[str, Any],
) -> np.ndarray:
    """Compute the intensity-weighted center (weighted centroid) of a cluster.

    Formula:
        c = sum(w_i * p_i) / sum(w_i)
    Where:
        I_clamped = min(I, percentile(I, clamp_percentile))
        w_i = max(I_clamped - threshold, 0.0) ^ intensity_weight_power

    Fallback:
        If total weight sum is zero, it falls back to simple centroid estimation.

    Args:
        cluster: The RFCluster candidate.
        threshold: The threshold value used to select high-reflectance points.
        cfg: Center estimation configuration dictionary.

    Returns:
        The estimated RF center coordinates with shape (3,).
    """
    points_xyz = cluster.points_xyz
    intensity = cluster.intensity

    if points_xyz.shape[0] == 0:
        return np.zeros(3, dtype=np.float64)

    try:
        power = float(cfg.get("intensity_weight_power", 1.0))
        clamp_percentile = float(cfg.get("clamp_percentile", 95.0))
    except (ValueError, TypeError) as exc:
        logger.error("Invalid center estimation configuration values.")
        raise ValueError(f"Invalid config values: {exc}") from exc

    # 1. Clamp intensity to prevent hot-spots from skewing center estimation
    clamp_val = np.percentile(intensity, clamp_percentile)
    intensity_clamped = np.minimum(intensity, clamp_val)

    # 2. Calculate weights
    weights = np.maximum(intensity_clamped - threshold, 0.0) ** power
    weight_sum = np.sum(weights)

    # 3. Handle zero weight sum fallback to simple centroid
    if weight_sum <= EPSILON:
        logger.debug(
            "Weight sum is zero for cluster %d. Falling back to simple centroid.",
            cluster.cluster_id,
        )
        return estimate_centroid(points_xyz)

    # 4. Calculate weighted center
    weighted_sum = np.sum(points_xyz * weights[:, np.newaxis], axis=0)
    center = weighted_sum / weight_sum

    return center
