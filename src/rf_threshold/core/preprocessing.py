"""Module for preprocessing LiDAR point cloud frames."""

import logging
from typing import Any, Dict, Tuple

import numpy as np

from rf_threshold.core.frame import LidarFrame

logger = logging.getLogger("preprocessing")


def remove_invalid_points(frame: LidarFrame) -> LidarFrame:
    """Remove NaN and Inf values from coordinates and intensity.

    Args:
        frame: The raw LidarFrame.

    Returns:
        A new LidarFrame containing only valid points with finite coordinates
        and intensity.
    """
    points_xyz = frame.points_xyz
    intensity = frame.intensity

    # Mask for points where all coordinates are finite and intensity is finite
    finite_mask = np.all(np.isfinite(points_xyz), axis=1) & np.isfinite(intensity)

    num_removed = points_xyz.shape[0] - np.sum(finite_mask)
    if num_removed > 0:
        logger.debug("Removed %d invalid (NaN/Inf) points.", num_removed)

    return LidarFrame(
        stamp=frame.stamp,
        frame_id=frame.frame_id,
        points_xyz=points_xyz[finite_mask],
        intensity=intensity[finite_mask],
    )


def preprocess_frame(
    frame: LidarFrame,
    cfg: Dict[str, Any],
) -> Tuple[LidarFrame, Dict[str, int]]:
    """Preprocess a LiDAR frame by applying various filters.

    Filters are applied in the following order:
    1. Invalid points removal (NaN/Inf)
    2. Range filtering (distance limit)
    3. Height filtering (z-coordinate limit)

    Args:
        frame: The raw input LidarFrame.
        cfg: Preprocessing configuration dictionary.

    Returns:
        A tuple containing:
        - The preprocessed LidarFrame.
        - A summary dictionary mapping processing stages to remaining point counts.

    Raises:
        KeyError: If required configuration keys are missing when a filter is enabled.
    """
    raw_count = frame.points_xyz.shape[0]
    current_frame = frame

    # 1. NaN and Inf filter
    if cfg.get("remove_nan", True):
        current_frame = remove_invalid_points(current_frame)
    valid_count = current_frame.points_xyz.shape[0]

    # 2. Range filter
    range_cfg = cfg.get("range_filter", {})
    if range_cfg.get("enabled", False):
        try:
            min_range = range_cfg["min_range"]
            max_range = range_cfg["max_range"]
        except KeyError as exc:
            logger.error("Missing required range_filter configuration key.")
            raise KeyError("Missing range_filter: min_range or max_range") from exc

        xyz = current_frame.points_xyz
        ranges = np.linalg.norm(xyz, axis=1)
        range_mask = (ranges >= min_range) & (ranges <= max_range)

        current_frame = LidarFrame(
            stamp=current_frame.stamp,
            frame_id=current_frame.frame_id,
            points_xyz=xyz[range_mask],
            intensity=current_frame.intensity[range_mask],
        )
    range_count = current_frame.points_xyz.shape[0]

    # 3. Height filter
    height_cfg = cfg.get("height_filter", {})
    if height_cfg.get("enabled", False):
        try:
            min_z = height_cfg["min_z"]
            max_z = height_cfg["max_z"]
        except KeyError as exc:
            logger.error("Missing required height_filter configuration key.")
            raise KeyError("Missing height_filter: min_z or max_z") from exc

        xyz = current_frame.points_xyz
        z_mask = (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)

        current_frame = LidarFrame(
            stamp=current_frame.stamp,
            frame_id=current_frame.frame_id,
            points_xyz=xyz[z_mask],
            intensity=current_frame.intensity[z_mask],
        )
    height_count = current_frame.points_xyz.shape[0]

    summary = {
        "raw_points": raw_count,
        "valid_points": valid_count,
        "range_filtered_points": range_count,
        "height_filtered_points": height_count,
    }

    logger.debug(
        "Preprocessing complete: raw=%d, valid=%d, range=%d, height=%d",
        raw_count,
        valid_count,
        range_count,
        height_count,
    )

    return current_frame, summary
