"""Module for selecting high-intensity bright points from a LiDAR frame."""

import logging
from typing import Any, Dict, Tuple

import numpy as np

from rf_threshold.core.frame import LidarFrame

logger = logging.getLogger("thresholding")


def compute_adaptive_threshold(intensity: np.ndarray, cfg: Dict[str, Any]) -> float:
    """Compute the adaptive intensity threshold for a frame.

    Formula:
        T_I = max(min_intensity, percentile(intensity, p))

    Args:
        intensity: Array of intensity values with shape (N,).
        cfg: Adaptive thresholding configuration.

    Returns:
        The computed adaptive threshold value.

    Raises:
        KeyError: If required configuration keys are missing.
    """
    if intensity.size == 0:
        return float(cfg["min_intensity"])

    try:
        percentile = cfg["percentile"]
        min_intensity = cfg["min_intensity"]
    except KeyError as exc:
        logger.error("Missing required adaptive thresholding configuration key.")
        raise KeyError("Missing adaptive keys: percentile or min_intensity") from exc

    pct_value = np.percentile(intensity, percentile)
    return float(max(min_intensity, pct_value))


def select_bright_points(
    frame: LidarFrame,
    cfg: Dict[str, Any],
) -> Tuple[LidarFrame, float]:
    """Select points with intensity values above a threshold.

    Supports both "fixed" and "adaptive" thresholding modes.

    Args:
        frame: The preprocessed input LidarFrame.
        cfg: Thresholding configuration dictionary.

    Returns:
        A tuple containing:
        - A new LidarFrame containing only the selected bright points.
        - The actual threshold value used for selection.

    Raises:
        ValueError: If an unsupported threshold mode is specified.
        KeyError: If required configuration keys are missing.
    """
    if frame.points_xyz.shape[0] == 0:
        logger.debug("Input frame is empty. Returning empty bright frame.")
        return LidarFrame(
            stamp=frame.stamp,
            frame_id=frame.frame_id,
            points_xyz=np.empty((0, 3), dtype=np.float64),
            intensity=np.empty((0,), dtype=np.float64),
        ), 0.0

    try:
        mode = cfg["mode"]
    except KeyError as exc:
        logger.error("Missing threshold 'mode' in configuration.")
        raise KeyError("Missing threshold: mode") from exc

    # Determine threshold value based on mode
    if mode == "fixed":
        try:
            threshold_val = float(cfg["fixed_intensity"])
        except KeyError as exc:
            logger.error("Missing fixed_intensity in configuration.")
            raise KeyError("Missing threshold: fixed_intensity") from exc

    elif mode == "adaptive":
        try:
            adaptive_cfg = cfg["adaptive"]
        except KeyError as exc:
            logger.error("Missing adaptive threshold configuration block.")
            raise KeyError("Missing threshold: adaptive") from exc
        threshold_val = compute_adaptive_threshold(frame.intensity, adaptive_cfg)

    else:
        logger.error("Unsupported threshold mode specified: %s", mode)
        raise ValueError(f"Unsupported threshold mode: {mode}")

    # Select points exceeding threshold
    bright_mask = frame.intensity >= threshold_val
    bright_points = frame.points_xyz[bright_mask]
    bright_intensity = frame.intensity[bright_mask]

    logger.debug(
        "Threshold selected %d/%d bright points using threshold=%.1f",
        bright_points.shape[0],
        frame.points_xyz.shape[0],
        threshold_val,
    )

    bright_frame = LidarFrame(
        stamp=frame.stamp,
        frame_id=frame.frame_id,
        points_xyz=bright_points,
        intensity=bright_intensity,
    )

    return bright_frame, threshold_val
