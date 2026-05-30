"""Module for validating the geometric distribution of matched landmark-detection pairs."""

from dataclasses import dataclass
import logging
from typing import Any, Dict, List, Optional
import numpy as np

from rf_threshold.localization.pose import MatchedPair

logger = logging.getLogger("geometry_check")


@dataclass(frozen=True)
class GeometryCheckResult:
    """Outcome of the geometric validation check.

    Args:
        is_valid: True if SVD registration is permitted to proceed.
        is_degenerate: True if collinear or collocated points are detected.
        warning: Optional warning label (e.g., "NEAR_COLLINEAR").
        reason: Text description of the state.
        num_pairs: Number of matched pairs evaluated.
        condition_number_lidar: Condition number of LiDAR matched points, if computed.
        condition_number_map: Condition number of Map matched points, if computed.
        spread_lidar: Spatial spread of LiDAR points in meters.
        spread_map: Spatial spread of Map points in meters.
        debug_info: Diagnostics dict.
    """
    is_valid: bool
    is_degenerate: bool
    warning: Optional[str]
    reason: str
    num_pairs: int
    condition_number_lidar: Optional[float]
    condition_number_map: Optional[float]
    spread_lidar: float
    spread_map: float
    debug_info: Dict[str, Any]

    def __post_init__(self) -> None:
        """Validate typing for Python 3.8 compatibility."""
        if not isinstance(self.is_valid, bool):
            raise TypeError("is_valid must be a boolean")
        if not isinstance(self.is_degenerate, bool):
            raise TypeError("is_degenerate must be a boolean")
        if self.warning is not None and not isinstance(self.warning, str):
            raise TypeError("warning must be a string or None")
        if not isinstance(self.reason, str):
            raise TypeError("reason must be a string")
        if not isinstance(self.num_pairs, int):
            raise TypeError("num_pairs must be an integer")
        if not isinstance(self.debug_info, dict):
            raise TypeError("debug_info must be a dictionary")


def check_geometry_validity(
    matched_pairs: List[MatchedPair],
    cfg: Dict[str, Any],
) -> GeometryCheckResult:
    """Evaluate the spatial and geometric layout of matched landmark-detection pairs.

    Enforces minimum point counts, checks for duplicates, finite values, spatial
    spread limits, and evaluates the condition number (collinearity) under warning-only rules.

    Args:
        matched_pairs: List of matches between observed and map landmarks.
        cfg: Configuration dictionary containing "geometry_check" parameters.

    Returns:
        A GeometryCheckResult special structure.
    """
    # 1. Load config
    geom_cfg = cfg.get("geometry_check", {})
    min_matches = geom_cfg.get("min_matches", 3)
    min_spread = geom_cfg.get("min_spread", 0.30)

    cond_cfg = geom_cfg.get("condition_number", {})
    cond_enabled = cond_cfg.get("enabled", True)
    max_condition_number = cond_cfg.get("max_condition_number", 50.0)
    hard_reject = cond_cfg.get("hard_reject", False)

    num_pairs = len(matched_pairs)

    # 2. Hard Reject: Minimum points
    if num_pairs < min_matches:
        msg = f"Insufficient matched pairs: {num_pairs} < {min_matches}"
        logger.warning(msg)
        return GeometryCheckResult(
            is_valid=False,
            is_degenerate=True,
            warning="INSUFFICIENT_POINTS",
            reason=msg,
            num_pairs=num_pairs,
            condition_number_lidar=None,
            condition_number_map=None,
            spread_lidar=0.0,
            spread_map=0.0,
            debug_info={},
        )

    # 3. Hard Reject: Duplicate detection_id
    det_ids = [m.detection_id for m in matched_pairs]
    if len(det_ids) != len(set(det_ids)):
        msg = "Duplicate detection IDs found in matched pairs."
        logger.warning(msg)
        return GeometryCheckResult(
            is_valid=False,
            is_degenerate=True,
            warning="DUPLICATE_IDS",
            reason=msg,
            num_pairs=num_pairs,
            condition_number_lidar=None,
            condition_number_map=None,
            spread_lidar=0.0,
            spread_map=0.0,
            debug_info={"duplicate_detection_ids": True},
        )

    # 4. Hard Reject: Duplicate landmark_id
    landmark_ids = [m.landmark_id for m in matched_pairs]
    if len(landmark_ids) != len(set(landmark_ids)):
        msg = "Duplicate landmark IDs found in matched pairs."
        logger.warning(msg)
        return GeometryCheckResult(
            is_valid=False,
            is_degenerate=True,
            warning="DUPLICATE_IDS",
            reason=msg,
            num_pairs=num_pairs,
            condition_number_lidar=None,
            condition_number_map=None,
            spread_lidar=0.0,
            spread_map=0.0,
            debug_info={"duplicate_landmark_ids": True},
        )

    # Extract 2D coordinate lists
    pts_lidar = np.array([m.point_lidar[:2] for m in matched_pairs], dtype=np.float64)
    pts_map = np.array([m.point_map[:2] for m in matched_pairs], dtype=np.float64)

    # 5. Hard Reject: Finite coordinate checks
    if not np.all(np.isfinite(pts_lidar)) or not np.all(np.isfinite(pts_map)):
        msg = "Coordinates contain non-finite values (NaN or Inf)."
        logger.warning(msg)
        return GeometryCheckResult(
            is_valid=False,
            is_degenerate=True,
            warning="INVALID_COORDINATES",
            reason=msg,
            num_pairs=num_pairs,
            condition_number_lidar=None,
            condition_number_map=None,
            spread_lidar=0.0,
            spread_map=0.0,
            debug_info={},
        )

    # 6. Hard Reject: Spatial spread check
    max_lidar = np.max(pts_lidar, axis=0)
    min_lidar = np.min(pts_lidar, axis=0)
    spread_lidar = float(np.linalg.norm(max_lidar - min_lidar))

    max_map = np.max(pts_map, axis=0)
    min_map = np.min(pts_map, axis=0)
    spread_map = float(np.linalg.norm(max_map - min_map))

    if spread_lidar < min_spread or spread_map < min_spread:
        msg = f"Spatial spread too small: lidar={spread_lidar:.3f}m, map={spread_map:.3f}m (min={min_spread}m)"
        logger.warning(msg)
        return GeometryCheckResult(
            is_valid=False,
            is_degenerate=True,
            warning="DEGENERATE_GEOMETRY",
            reason=msg,
            num_pairs=num_pairs,
            condition_number_lidar=None,
            condition_number_map=None,
            spread_lidar=spread_lidar,
            spread_map=spread_map,
            debug_info={},
        )

    # 7. Condition Number analysis
    cond_lidar = None
    cond_map = None

    if cond_enabled:
        # Centering points to calculate covariance shape
        centroid_lidar = np.mean(pts_lidar, axis=0)
        centered_lidar = pts_lidar - centroid_lidar
        _, s_lidar, _ = np.linalg.svd(centered_lidar)
        
        # Calculate condition number: S[0] / S[1]
        if len(s_lidar) >= 2 and s_lidar[1] > 1e-12:
            cond_lidar = float(s_lidar[0] / s_lidar[1])
        else:
            cond_lidar = float("inf")

        centroid_map = np.mean(pts_map, axis=0)
        centered_map = pts_map - centroid_map
        _, s_map, _ = np.linalg.svd(centered_map)
        
        if len(s_map) >= 2 and s_map[1] > 1e-12:
            cond_map = float(s_map[0] / s_map[1])
        else:
            cond_map = float("inf")

    debug_stats = {
        "condition_number_lidar": cond_lidar,
        "condition_number_map": cond_map,
        "spread_lidar": spread_lidar,
        "spread_map": spread_map,
    }

    # 8. Warning-only or Hard-reject collinear check
    is_collinear = False
    if cond_enabled:
        if (cond_lidar is not None and cond_lidar > max_condition_number) or \
           (cond_map is not None and cond_map > max_condition_number):
            is_collinear = True

    if is_collinear:
        msg = f"Near-collinear geometry exceeds threshold: cond_lidar={cond_lidar}, cond_map={cond_map}"
        if hard_reject:
            logger.warning("Rejecting collinear points (hard_reject enabled).")
            return GeometryCheckResult(
                is_valid=False,
                is_degenerate=True,
                warning="NEAR_COLLINEAR",
                reason=msg,
                num_pairs=num_pairs,
                condition_number_lidar=cond_lidar,
                condition_number_map=cond_map,
                spread_lidar=spread_lidar,
                spread_map=spread_map,
                debug_info=debug_stats,
            )
        else:
            logger.info("Near-collinear points accepted as a warning (default behavior).")
            return GeometryCheckResult(
                is_valid=True,
                is_degenerate=True,
                warning="NEAR_COLLINEAR",
                reason="Near-collinear geometry detected, continuing with warning.",
                num_pairs=num_pairs,
                condition_number_lidar=cond_lidar,
                condition_number_map=cond_map,
                spread_lidar=spread_lidar,
                spread_map=spread_map,
                debug_info=debug_stats,
            )

    # 9. Clear validation
    return GeometryCheckResult(
        is_valid=True,
        is_degenerate=False,
        warning=None,
        reason="Geometry validation passed successfully.",
        num_pairs=num_pairs,
        condition_number_lidar=cond_lidar,
        condition_number_map=cond_map,
        spread_lidar=spread_lidar,
        spread_map=spread_map,
        debug_info=debug_stats,
    )
