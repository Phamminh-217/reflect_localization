"""Core data structures for RF threshold-based localization."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class LidarFrame:
    """Represent one LiDAR point cloud frame.

    Args:
        stamp: ROS timestamp in seconds.
        frame_id: Coordinate frame name of the LiDAR data.
        points_xyz: Point coordinates with shape (N, 3).
        intensity: Intensity values with shape (N,).
    """

    stamp: float
    frame_id: str
    points_xyz: np.ndarray
    intensity: np.ndarray

    def __post_init__(self) -> None:
        """Validate array shapes after initialization."""
        if self.points_xyz.ndim != 2 or self.points_xyz.shape[1] != 3:
            raise ValueError(
                "points_xyz must have shape (N, 3), "
                f"got {self.points_xyz.shape}"
            )

        if self.intensity.ndim != 1:
            raise ValueError(
                "intensity must have shape (N,), "
                f"got {self.intensity.shape}"
            )

        if self.points_xyz.shape[0] != self.intensity.shape[0]:
            raise ValueError(
                "points_xyz and intensity must have the same number of points: "
                f"{self.points_xyz.shape[0]} != {self.intensity.shape[0]}"
            )


@dataclass(frozen=True)
class RFCluster:
    """Represent one candidate RF point cluster.

    Args:
        cluster_id: Cluster ID inside one LiDAR frame.
        point_indices: Indices of points belonging to this cluster.
        points_xyz: Cluster point coordinates with shape (N, 3).
        intensity: Cluster intensity values with shape (N,).
    """

    cluster_id: int
    point_indices: np.ndarray
    points_xyz: np.ndarray
    intensity: np.ndarray

    def __post_init__(self) -> None:
        """Validate cluster data."""
        if self.cluster_id < 0:
            raise ValueError(f"cluster_id must be non-negative, got {self.cluster_id}")

        if self.points_xyz.ndim != 2 or self.points_xyz.shape[1] != 3:
            raise ValueError(
                "points_xyz must have shape (N, 3), "
                f"got {self.points_xyz.shape}"
            )

        if self.intensity.ndim != 1:
            raise ValueError(
                "intensity must have shape (N,), "
                f"got {self.intensity.shape}"
            )

        if self.point_indices.ndim != 1:
            raise ValueError(
                "point_indices must have shape (N,), "
                f"got {self.point_indices.shape}"
            )

        num_points = self.points_xyz.shape[0]
        if self.intensity.shape[0] != num_points:
            raise ValueError(
                "points_xyz and intensity must have the same length: "
                f"{num_points} != {self.intensity.shape[0]}"
            )

        if self.point_indices.shape[0] != num_points:
            raise ValueError(
                "point_indices and points_xyz must have the same length: "
                f"{self.point_indices.shape[0]} != {num_points}"
            )


@dataclass(frozen=True)
class RFDetection:
    """Represent one accepted reflective landmark detection.

    Args:
        detection_id: Detection ID inside one LiDAR frame.
        stamp: ROS timestamp in seconds.
        frame_id: Coordinate frame name.
        center_lidar: Estimated RF center in LiDAR frame with shape (3,).
        score: Rule-based confidence score in [0, 1].
        num_points: Number of points in the accepted cluster.
        mean_intensity: Mean intensity of the cluster.
        max_intensity: Maximum intensity of the cluster.
        bbox_min: Minimum xyz values of the cluster bounding box.
        bbox_max: Maximum xyz values of the cluster bounding box.
        cluster_id: Optional source cluster ID.
    """

    detection_id: int
    stamp: float
    frame_id: str
    center_lidar: np.ndarray
    score: float
    num_points: int
    mean_intensity: float
    max_intensity: float
    bbox_min: np.ndarray
    bbox_max: np.ndarray
    cluster_id: Optional[int] = None

    def __post_init__(self) -> None:
        """Validate detection data."""
        if self.detection_id < 0:
            raise ValueError(
                f"detection_id must be non-negative, got {self.detection_id}"
            )

        if self.cluster_id is not None and self.cluster_id < 0:
            raise ValueError(f"cluster_id must be non-negative, got {self.cluster_id}")

        if self.center_lidar.shape != (3,):
            raise ValueError(
                "center_lidar must have shape (3,), "
                f"got {self.center_lidar.shape}"
            )

        if self.bbox_min.shape != (3,):
            raise ValueError(f"bbox_min must have shape (3,), got {self.bbox_min.shape}")

        if self.bbox_max.shape != (3,):
            raise ValueError(f"bbox_max must have shape (3,), got {self.bbox_max.shape}")

        if not 0.0 <= self.score <= 1.0:
            raise ValueError(f"score must be in [0, 1], got {self.score}")

        if self.num_points < 0:
            raise ValueError(f"num_points must be non-negative, got {self.num_points}")

        if not np.all(np.isfinite(self.center_lidar)):
            raise ValueError(f"center_lidar contains invalid values: {self.center_lidar}")
