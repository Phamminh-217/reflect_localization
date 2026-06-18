"""Localization module containing SVD pose estimation and mapping structures."""

from rf_threshold.localization.pose import (
    LocalizationResult,
    LocalizationStatus,
    MatchedPair,
    RFMapLandmark,
    RobotPose,
)
from rf_threshold.localization.map_loader import load_rf_map
from rf_threshold.localization.svd_pose import (
    estimate_pose_svd_2d,
    SVDPoseResult,
)
from rf_threshold.localization.data_association import (
    associate_detections_to_map,
    TripletDescriptor,
    AssociationCandidate,
    AssociationResult,
)
from rf_threshold.localization.localizer_pipeline import RFLocalizer
from rf_threshold.localization.detection_loader import (
    DetectionFrame,
    DetectionLoader,
)
from rf_threshold.localization.localization_writer import LocalizationWriter
from rf_threshold.localization.fallback_manager import (
    FallbackOutput,
    FallbackManager,
)
from rf_threshold.localization.pose_evaluator import (
    PoseEvalMetrics,
    evaluate_poses_from_csv,
    evaluate_poses_from_results,
)

__all__ = [
    "LocalizationResult",
    "LocalizationStatus",
    "MatchedPair",
    "RFMapLandmark",
    "RobotPose",
    "load_rf_map",
    "estimate_pose_svd_2d",
    "SVDPoseResult",
    "associate_detections_to_map",
    "TripletDescriptor",
    "AssociationCandidate",
    "AssociationResult",
    "RFLocalizer",
    "DetectionFrame",
    "DetectionLoader",
    "LocalizationWriter",
    "FallbackOutput",
    "FallbackManager",
    "PoseEvalMetrics",
    "evaluate_poses_from_csv",
    "evaluate_poses_from_results",
]




