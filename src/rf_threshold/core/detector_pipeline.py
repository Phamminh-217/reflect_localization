"""Pipeline coordinating all stages of the RF threshold detector."""

import logging
from typing import Any, Dict, List, Tuple

import numpy as np

from rf_threshold.core.center_estimation import estimate_intensity_weighted_center
from rf_threshold.core.cluster_validation import validate_cluster_with_reason
from rf_threshold.core.clustering import cluster_bright_points
from rf_threshold.core.frame import LidarFrame, RFDetection
from rf_threshold.core.preprocessing import preprocess_frame
from rf_threshold.core.thresholding import select_bright_points

logger = logging.getLogger("detector_pipeline")


class ThresholdRFDetector:
    """Threshold-based Reflective Feature (RF) detector pipeline."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        """Initialize the detector.

        Args:
            cfg: Full configuration dictionary loaded from YAML.
        """
        self.cfg = cfg

    def _calculate_score(
        self,
        num_points: int,
        mean_intensity: float,
        extent: np.ndarray,
    ) -> float:
        """Calculate a rule-based confidence score in [0, 1] for a detection.

        Formula:
            score = 0.5 * intensity_score + 0.3 * compactness_score + 0.2 * point_count_score

        Args:
            num_points: Number of points in the cluster.
            mean_intensity: Mean intensity of the cluster.
            extent: Bounding box extent along X, Y, and Z axes.

        Returns:
            The calculated confidence score in [0.0, 1.0].
        """
        # 1. Intensity score: relative to max intensity 255.0
        intensity_score = np.clip(mean_intensity / 255.0, 0.0, 1.0)

        # 2. Compactness score: high if the cluster is small (extent <= 0.30m)
        max_extent = np.max(extent[:2])  # check horizontal spread
        compactness_score = 1.0 - np.clip(max_extent / 0.30, 0.0, 1.0)

        # 3. Point count score: high if the point count is robust (e.g. 15-20 points)
        point_count_score = np.clip(num_points / 20.0, 0.0, 1.0)

        score = (
            0.5 * intensity_score
            + 0.3 * compactness_score
            + 0.2 * point_count_score
        )
        return float(np.clip(score, 0.0, 1.0))

    def detect(self, frame: LidarFrame) -> Tuple[List[RFDetection], Dict[str, Any]]:
        """Run the full detection pipeline on a single LiDAR frame.

        Pipeline Stages:
        1. Preprocessing (NaN removal, range/height filters)
        2. Thresholding (Fixed or Adaptive selection of bright points)
        3. DBSCAN Clustering (grouping bright points)
        4. Cluster Validation (geometric/intensity checks)
        5. RF Center Estimation (Intensity-weighted centroid calculation)
        6. Detection creation

        Args:
            frame: The raw input LidarFrame.

        Returns:
            A tuple containing:
            - A list of validated RFDetection objects.
            - A dictionary containing intermediate debug data (e.g. counts, thresholds).

        Raises:
            KeyError: If required configuration keys are missing.
        """
        # Stage 1: Preprocessing
        preprocessed_frame, prep_summary = preprocess_frame(
            frame, self.cfg.get("preprocessing", {})
        )

        # Stage 2: Thresholding
        threshold_cfg = self.cfg.get("threshold", {})
        bright_frame, threshold_val = select_bright_points(
            preprocessed_frame, threshold_cfg
        )

        # Stage 3: Clustering
        clustering_cfg = self.cfg.get("clustering", {})
        clusters = cluster_bright_points(bright_frame, clustering_cfg)

        # Stage 4 & 5: Validation & Center Estimation
        validation_cfg = self.cfg.get("cluster_validation", {})
        center_cfg = self.cfg.get("center_estimation", {})

        detections: List[RFDetection] = []
        rejected_list: List[Dict[str, Any]] = []

        detection_id = 0
        for cluster in clusters:
            is_valid, reason = validate_cluster_with_reason(cluster, validation_cfg)

            # Bounding box calculation for scoring and reporting
            bbox_min = np.min(cluster.points_xyz, axis=0)
            bbox_max = np.max(cluster.points_xyz, axis=0)
            extent = bbox_max - bbox_min
            mean_intensity = np.mean(cluster.intensity)

            if is_valid:
                # Estimate the weighted center
                center_lidar = estimate_intensity_weighted_center(
                    cluster, threshold_val, center_cfg
                )

                # Compute rule-based score
                score = self._calculate_score(
                    len(cluster.points_xyz),
                    mean_intensity,
                    extent,
                )

                detection = RFDetection(
                    detection_id=detection_id,
                    stamp=frame.stamp,
                    frame_id=frame.frame_id,
                    center_lidar=center_lidar,
                    score=score,
                    num_points=len(cluster.points_xyz),
                    mean_intensity=mean_intensity,
                    max_intensity=float(np.max(cluster.intensity)),
                    bbox_min=bbox_min,
                    bbox_max=bbox_max,
                    cluster_id=cluster.cluster_id,
                )
                detections.append(detection)
                detection_id += 1
            else:
                rejected_list.append(
                    {
                        "cluster_id": cluster.cluster_id,
                        "reason": reason,
                        "num_points": len(cluster.points_xyz),
                        "extent_x": extent[0],
                        "extent_y": extent[1],
                        "mean_intensity": mean_intensity,
                    }
                )

        debug_data = {
            "preprocessing_summary": prep_summary,
            "threshold_value": threshold_val,
            "bright_points_count": bright_frame.points_xyz.shape[0],
            "num_clusters": len(clusters),
            "rejected_clusters": rejected_list,
            "preprocessed_frame": preprocessed_frame,
            "bright_frame": bright_frame,
            "clusters": [
                c
                for c in clusters
                if c.cluster_id not in [r["cluster_id"] for r in rejected_list]
            ],
        }

        return detections, debug_data
