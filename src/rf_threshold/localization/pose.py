"""Data structures for SVD-based robot localization."""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional
import numpy as np


class LocalizationStatus(Enum):
    """Status enum for the localization pipeline."""
    OK = "OK"
    INSUFFICIENT_DETECTIONS = "INSUFFICIENT_DETECTIONS"
    INSUFFICIENT_MATCHES = "INSUFFICIENT_MATCHES"
    DEGENERATE_GEOMETRY = "DEGENERATE_GEOMETRY"
    HIGH_RESIDUAL = "HIGH_RESIDUAL"
    MAP_ERROR = "MAP_ERROR"
    ASSOCIATION_FAILED = "ASSOCIATION_FAILED"
    FALLBACK_LAST_VALID_POSE = "FALLBACK_LAST_VALID_POSE"
    ODOM_FALLBACK = "ODOM_FALLBACK"
    ERROR = "ERROR"


@dataclass(frozen=True)
class RFMapLandmark:
    """Represents a reflective feature landmark in the map coordinate frame.

    Args:
        landmark_id: A unique non-negative identifier.
        position_map: 3D coordinates [x, y, z] in map frame.
        frame_id: Name of the coordinate system.
    """
    landmark_id: int
    position_map: np.ndarray
    frame_id: str = "map_frame"

    def __post_init__(self) -> None:
        """Validate landmark data."""
        if self.landmark_id < 0:
            raise ValueError(f"landmark_id must be non-negative, got {self.landmark_id}")
        
        if self.position_map.shape != (3,):
            raise ValueError(f"position_map must have shape (3,), got {self.position_map.shape}")
            
        if not np.all(np.isfinite(self.position_map)):
            raise ValueError("position_map contains NaN or Inf values")


@dataclass(frozen=True)
class MatchedPair:
    """Represents a matched detection-landmark pair.

    Args:
        detection_id: Detection identifier.
        landmark_id: Map landmark identifier.
        point_lidar: 3D point in LiDAR frame.
        point_map: 3D point in Map frame.
        weight: Positive weighting parameter for localization.
    """
    detection_id: int
    landmark_id: int
    point_lidar: np.ndarray
    point_map: np.ndarray
    weight: float = 1.0

    def __post_init__(self) -> None:
        """Validate match pair data."""
        if self.detection_id < 0:
            raise ValueError(f"detection_id must be non-negative, got {self.detection_id}")
        if self.landmark_id < 0:
            raise ValueError(f"landmark_id must be non-negative, got {self.landmark_id}")
            
        if self.point_lidar.shape != (3,):
            raise ValueError(f"point_lidar must have shape (3,), got {self.point_lidar.shape}")
        if self.point_map.shape != (3,):
            raise ValueError(f"point_map must have shape (3,), got {self.point_map.shape}")
            
        if not np.all(np.isfinite(self.point_lidar)):
            raise ValueError("point_lidar contains NaN or Inf values")
        if not np.all(np.isfinite(self.point_map)):
            raise ValueError("point_map contains NaN or Inf values")
            
        if self.weight <= 0.0:
            raise ValueError(f"weight must be strictly positive, got {self.weight}")
        if not np.isfinite(self.weight):
            raise ValueError("weight must be finite")


@dataclass(frozen=True)
class RobotPose:
    """Represents the estimated 2D/3D robot pose.

    Args:
        stamp: Timestamp in seconds.
        frame_id: Coordinate frame of the pose (e.g. "map_frame").
        child_frame_id: Child frame of the pose (e.g. "lidar_frame").
        x: Estimated X coordinate in meters.
        y: Estimated Y coordinate in meters.
        yaw: Estimated rotation about Z axis in radians.
        residual_rmse: RMSE of the SVD registration process.
        num_matches: Number of matching pairs used.
    """
    stamp: float
    frame_id: str
    child_frame_id: str
    x: float
    y: float
    yaw: float
    residual_rmse: float
    num_matches: int

    def __post_init__(self) -> None:
        """Validate pose data."""
        if not np.isfinite(self.x):
            raise ValueError(f"x must be finite, got {self.x}")
        if not np.isfinite(self.y):
            raise ValueError(f"y must be finite, got {self.y}")
        if not np.isfinite(self.yaw):
            raise ValueError(f"yaw must be finite, got {self.yaw}")
        if not np.isfinite(self.residual_rmse):
            raise ValueError(f"residual_rmse must be finite, got {self.residual_rmse}")
        if self.residual_rmse < 0.0:
            raise ValueError(f"residual_rmse must be non-negative, got {self.residual_rmse}")
        if self.num_matches < 0:
            raise ValueError(f"num_matches must be non-negative, got {self.num_matches}")


@dataclass(frozen=True)
class LocalizationResult:
    """Combines localization pose, status, matches, and metadata.

    Args:
        stamp: Timestamp in seconds.
        status: The pipeline outcome status.
        pose: The computed RobotPose, if successful.
        matched_pairs: List of matched associations.
        residual_rmse: Registration RMSE if applicable.
        reason: Text description of state/errors.
        debug_info: Diagnostics dict.
    """
    stamp: float
    status: LocalizationStatus
    pose: Optional[RobotPose]
    matched_pairs: List[MatchedPair]
    residual_rmse: Optional[float]
    reason: str
    debug_info: Dict[str, Any]

    def __post_init__(self) -> None:
        """Validate localization outcome."""
        if self.status == LocalizationStatus.OK and self.pose is None:
            raise ValueError("pose cannot be None when status is OK")
            
        if self.pose is not None:
            if not np.isfinite(self.pose.residual_rmse):
                raise ValueError("pose.residual_rmse must be finite")
                
        if not isinstance(self.matched_pairs, list):
            raise TypeError("matched_pairs must be a list")
            
        if not isinstance(self.debug_info, dict):
            raise TypeError("debug_info must be a dict")
