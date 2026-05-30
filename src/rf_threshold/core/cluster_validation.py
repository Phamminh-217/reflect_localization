"""Module for validating RF cluster candidates using geometric and intensity rules."""

import logging
from typing import Any, Dict, Tuple

import numpy as np

from rf_threshold.core.frame import RFCluster

logger = logging.getLogger("cluster_validation")


def validate_cluster_with_reason(
    cluster: RFCluster,
    cfg: Dict[str, Any],
) -> Tuple[bool, str]:
    """Validate an RFCluster candidate against geometric and intensity criteria.

    Criteria include:
    1. Point count limits (min_points <= count <= max_points)
    2. Geometric extent limits along X, Y, and Z axes
    3. Minimum mean intensity limit

    Args:
        cluster: The RFCluster candidate to validate.
        cfg: Validation configuration dictionary.

    Returns:
        A tuple containing:
        - A boolean indicating if the cluster is valid.
        - A string code representing the rejection reason (or "valid" if accepted).

    Raises:
        KeyError: If required validation keys are missing from the configuration.
    """
    try:
        min_points = int(cfg["min_points"])
        max_points = int(cfg["max_points"])
        max_extent_x = float(cfg["max_extent_x"])
        max_extent_y = float(cfg["max_extent_y"])
        max_extent_z = float(cfg["max_extent_z"])
        min_mean_intensity = float(cfg["min_mean_intensity"])
    except KeyError as exc:
        logger.error("Missing required cluster validation configuration key.")
        raise KeyError(
            "Missing validation keys: min_points, max_points, "
            "max_extent_x, max_extent_y, max_extent_z, or min_mean_intensity"
        ) from exc

    points_xyz = cluster.points_xyz
    num_points = points_xyz.shape[0]

    # 1. Point count check
    if num_points < min_points:
        return False, "num_points_too_small"
    if num_points > max_points:
        return False, "num_points_too_large"

    # Calculate bounding box dimensions
    bbox_min = np.min(points_xyz, axis=0)
    bbox_max = np.max(points_xyz, axis=0)
    extent = bbox_max - bbox_min

    # 2. Geometric extent checks
    if extent[0] > max_extent_x:
        return False, "extent_x_too_large"
    if extent[1] > max_extent_y:
        return False, "extent_y_too_large"
    if extent[2] > max_extent_z:
        return False, "extent_z_too_large"

    # 3. Intensity check
    mean_intensity = np.mean(cluster.intensity)
    if mean_intensity < min_mean_intensity:
        return False, "mean_intensity_too_low"

    return True, "valid"
