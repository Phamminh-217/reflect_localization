"""Module for the core localization pipeline orchestrating mapping, association, and registration."""

import logging
from typing import Any, Dict, List, Optional
import numpy as np

from rf_threshold.core.frame import RFDetection
from rf_threshold.localization.map_loader import load_rf_map
from rf_threshold.localization.data_association import associate_detections_to_map
from rf_threshold.localization.geometry_check import check_geometry_validity
from rf_threshold.localization.svd_pose import estimate_pose_svd_2d
from rf_threshold.localization.pose import (
    LocalizationResult,
    LocalizationStatus,
    MatchedPair,
    RobotPose,
)

logger = logging.getLogger("localizer_pipeline")


class RFLocalizer:
    """Orchestrator for SVD-based reflective feature (RF) landmark localization."""

    def __init__(self, map_path: str, cfg: Dict[str, Any]) -> None:
        """Initialize the localizer by loading the global RF map.

        Args:
            map_path: Path to the JSON map landmarks file.
            cfg: Configuration dictionary.
        """
        self.cfg = cfg
        self.landmarks = load_rf_map(map_path)
        logger.info("RFLocalizer initialized with %d landmarks", len(self.landmarks))

    def localize(self, detections: List[RFDetection], stamp: float) -> LocalizationResult:
        """Execute the localization pipeline for a single frame.

        Orchestrates:
            1. Detections count filtering and validation.
            2. Data association matching observed points to map frame landmarks.
            3. Geometry verification checking duplicates, spread, and near-collinear warnings.
            4. Final SVD pose registration solving SE(2) robot transformation.
            5. Residual verification validating RMSE limits.

        Args:
            detections: List of observed RF detections in lidar frame.
            stamp: Frame timestamp in seconds.

        Returns:
            A LocalizationResult containing the estimated RobotPose and diagnostic outcomes.
        """
        # 1. Base input count filtering check
        da_cfg = self.cfg.get("data_association", {})
        min_detections = da_cfg.get("min_detections", 3)

        if len(detections) < min_detections:
            msg = f"Insufficient detections: {len(detections)} < {min_detections}"
            logger.warning(msg)
            return LocalizationResult(
                stamp=stamp,
                status=LocalizationStatus.INSUFFICIENT_DETECTIONS,
                pose=None,
                matched_pairs=[],
                residual_rmse=None,
                reason=msg,
                debug_info={"num_detections": len(detections)},
            )

        # 2. Execute Data Association
        assoc_res = associate_detections_to_map(detections, self.landmarks, self.cfg)
        if assoc_res.status != LocalizationStatus.OK:
            logger.warning("Data association failed: %s", assoc_res.reason)
            return LocalizationResult(
                stamp=stamp,
                status=assoc_res.status,
                pose=None,
                matched_pairs=[],
                residual_rmse=None,
                reason=assoc_res.reason,
                debug_info=assoc_res.debug_info,
            )

        # 3. Geometry Check Validation
        geom_res = check_geometry_validity(assoc_res.matched_pairs, self.cfg)
        if not geom_res.is_valid:
            logger.warning("Geometric validation failed: %s", geom_res.reason)
            return LocalizationResult(
                stamp=stamp,
                status=LocalizationStatus.DEGENERATE_GEOMETRY,
                pose=None,
                matched_pairs=assoc_res.matched_pairs,
                residual_rmse=assoc_res.residual_rmse,
                reason=geom_res.reason,
                debug_info=geom_res.debug_info,
            )

        # 4. Final SVD Pose Estimation
        pts_lidar = np.array([m.point_lidar[:2] for m in assoc_res.matched_pairs], dtype=np.float64)
        pts_map = np.array([m.point_map[:2] for m in assoc_res.matched_pairs], dtype=np.float64)

        try:
            final_svd = estimate_pose_svd_2d(pts_lidar, pts_map)
        except ValueError as exc:
            msg = f"Final SVD registration failed: {exc}"
            logger.error(msg)
            return LocalizationResult(
                stamp=stamp,
                status=LocalizationStatus.ERROR,
                pose=None,
                matched_pairs=assoc_res.matched_pairs,
                residual_rmse=None,
                reason=msg,
                debug_info={},
            )

        # 5. Final Residual Verification
        max_candidate_rmse = da_cfg.get("max_candidate_rmse", 0.08)
        if final_svd.residual_rmse > max_candidate_rmse:
            msg = f"Final RMSE exceeds limit: {final_svd.residual_rmse:.3f}m > {max_candidate_rmse}m"
            logger.warning(msg)
            return LocalizationResult(
                stamp=stamp,
                status=LocalizationStatus.HIGH_RESIDUAL,
                pose=None,
                matched_pairs=assoc_res.matched_pairs,
                residual_rmse=final_svd.residual_rmse,
                reason=msg,
                debug_info={},
            )

        # 6. Construct RobotPose
        pose = RobotPose(
            stamp=stamp,
            frame_id="map_frame",
            child_frame_id="lidar_frame",
            x=float(final_svd.t[0]),
            y=float(final_svd.t[1]),
            yaw=float(final_svd.yaw),
            residual_rmse=float(final_svd.residual_rmse),
            num_matches=final_svd.num_points,
        )

        # 7. Success (preserving near-collinear warnings)
        reason = "Localization succeeded."
        if geom_res.warning == "NEAR_COLLINEAR":
            reason = "Near-collinear geometry warning: Pose estimated successfully."
            logger.info("Pose resolved under near-collinear warning: rmse=%.3fm", final_svd.residual_rmse)

        logger.info(
            "Localization succeeded: frame=%.3f x=%.3fm, y=%.3fm, yaw=%.3frad, rmse=%.3fm",
            stamp, pose.x, pose.y, pose.yaw, final_svd.residual_rmse,
        )

        # Combine debug stats
        debug_output = {
            "geom_check": geom_res.debug_info,
            "assoc_check": assoc_res.debug_info,
        }

        return LocalizationResult(
            stamp=stamp,
            status=LocalizationStatus.OK,
            pose=pose,
            matched_pairs=assoc_res.matched_pairs,
            residual_rmse=final_svd.residual_rmse,
            reason=reason,
            debug_info=debug_output,
        )
