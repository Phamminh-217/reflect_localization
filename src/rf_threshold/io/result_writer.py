"""Module for writing RF detection results to JSON and CSV formats."""

import csv
import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Union

import numpy as np

from rf_threshold.core.frame import RFDetection

logger = logging.getLogger("result_writer")


class ResultWriter:
    """Class to accumulate and write detection results and summaries."""

    def __init__(self, output_dir: Union[str, Path]) -> None:
        """Initialize the result writer.

        Args:
            output_dir: Directory where results should be saved.
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.frames_summary: List[Dict[str, Any]] = []
        self.detections_list: List[Dict[str, Any]] = []
        self.rejected_clusters: List[Dict[str, Any]] = []

    def add_frame_results(
        self,
        frame_index: int,
        stamp: float,
        frame_id: str,
        preprocessing_summary: Dict[str, int],
        threshold: float,
        bright_points: int,
        num_clusters: int,
        valid_detections: List[RFDetection],
        rejected_list: List[Dict[str, Any]],
    ) -> None:
        """Accumulate results from a single frame.

        Args:
            frame_index: Index of the current frame.
            stamp: Timestamp of the current frame in seconds.
            frame_id: Name of the LiDAR coordinate frame.
            preprocessing_summary: Dictionary of preprocessed point counts.
            threshold: Threshold intensity value used.
            bright_points: Number of bright points found.
            num_clusters: Total number of DBSCAN clusters.
            valid_detections: List of validated RFDetection objects.
            rejected_list: List of dictionaries describing rejected clusters.
        """
        # 1. Accumulate frame summary
        self.frames_summary.append(
            {
                "frame_index": frame_index,
                "stamp": stamp,
                "raw_points": preprocessing_summary.get("raw_points", 0),
                "preprocessed_points": preprocessing_summary.get(
                    "height_filtered_points", 0
                ),
                "bright_points": bright_points,
                "num_clusters": num_clusters,
                "num_valid": len(valid_detections),
                "threshold": threshold,
            }
        )

        # 2. Accumulate valid detections for JSON export
        frame_objects = []
        for det in valid_detections:
            frame_objects.append(
                {
                    "detection_id": det.detection_id,
                    "center_lidar": det.center_lidar.tolist(),
                    "score": float(det.score),
                    "num_points": int(det.num_points),
                    "mean_intensity": float(det.mean_intensity),
                    "max_intensity": float(det.max_intensity),
                    "bbox_min": det.bbox_min.tolist(),
                    "bbox_max": det.bbox_max.tolist(),
                }
            )

        self.detections_list.append(
            {
                "frame_index": frame_index,
                "stamp": stamp,
                "frame_id": frame_id,
                "num_valid_detections": len(valid_detections),
                "objects": frame_objects,
            }
        )

        # 3. Accumulate rejected clusters
        for rej in rejected_list:
            self.rejected_clusters.append(
                {
                    "frame_index": frame_index,
                    "cluster_id": rej["cluster_id"],
                    "reason": rej["reason"],
                    "num_points": rej["num_points"],
                    "extent_x": float(rej["extent_x"]),
                    "extent_y": float(rej["extent_y"]),
                    "mean_intensity": float(rej["mean_intensity"]),
                }
            )

    def write_results(self) -> None:
        """Write all accumulated results to JSON and CSV files."""
        # 1. Save detections.json
        json_path = self.output_dir / "detections.json"
        try:
            with json_path.open("w", encoding="utf-8") as file:
                json.dump({"detections": self.detections_list}, file, indent=2)
            logger.info("Saved detections JSON to: %s", json_path)
        except Exception as exc:
            logger.error("Failed to write detections.json: %s", exc)
            raise

        # 2. Save detections.csv
        csv_det_path = self.output_dir / "detections.csv"
        try:
            with csv_det_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "frame_index",
                        "stamp",
                        "detection_id",
                        "x_lidar",
                        "y_lidar",
                        "z_lidar",
                        "score",
                        "num_points",
                        "mean_intensity",
                        "max_intensity",
                    ]
                )
                for frame in self.detections_list:
                    for obj in frame["objects"]:
                        center = obj["center_lidar"]
                        writer.writerow(
                            [
                                frame["frame_index"],
                                f"{frame['stamp']:.6f}",
                                obj["detection_id"],
                                f"{center[0]:.4f}",
                                f"{center[1]:.4f}",
                                f"{center[2]:.4f}",
                                f"{obj['score']:.4f}",
                                obj["num_points"],
                                f"{obj['mean_intensity']:.2f}",
                                f"{obj['max_intensity']:.2f}",
                            ]
                        )
            logger.info("Saved detections CSV to: %s", csv_det_path)
        except Exception as exc:
            logger.error("Failed to write detections.csv: %s", exc)
            raise

        # 3. Save frame_summary.csv
        csv_sum_path = self.output_dir / "frame_summary.csv"
        try:
            with csv_sum_path.open("w", encoding="utf-8", newline="") as file:
                if self.frames_summary:
                    fields = list(self.frames_summary[0].keys())
                    writer = csv.DictWriter(file, fieldnames=fields)
                    writer.writeheader()
                    for row in self.frames_summary:
                        # Format stamp to 6 decimal places
                        formatted_row = row.copy()
                        formatted_row["stamp"] = f"{row['stamp']:.6f}"
                        formatted_row["threshold"] = f"{row['threshold']:.2f}"
                        writer.writerow(formatted_row)
            logger.info("Saved frame summary CSV to: %s", csv_sum_path)
        except Exception as exc:
            logger.error("Failed to write frame_summary.csv: %s", exc)
            raise

        # 4. Save rejected_clusters.csv
        csv_rej_path = self.output_dir / "rejected_clusters.csv"
        try:
            with csv_rej_path.open("w", encoding="utf-8", newline="") as file:
                if self.rejected_clusters:
                    fields = list(self.rejected_clusters[0].keys())
                    writer = csv.DictWriter(file, fieldnames=fields)
                    writer.writeheader()
                    for row in self.rejected_clusters:
                        formatted_row = row.copy()
                        formatted_row["extent_x"] = f"{row['extent_x']:.4f}"
                        formatted_row["extent_y"] = f"{row['extent_y']:.4f}"
                        formatted_row["mean_intensity"] = f"{row['mean_intensity']:.2f}"
                        writer.writerow(formatted_row)
                else:
                    # Write header anyway
                    writer = csv.writer(file)
                    writer.writerow(
                        [
                            "frame_index",
                            "cluster_id",
                            "reason",
                            "num_points",
                            "extent_x",
                            "extent_y",
                            "mean_intensity",
                        ]
                    )
            logger.info("Saved rejected clusters CSV to: %s", csv_rej_path)
        except Exception as exc:
            logger.error("Failed to write rejected_clusters.csv: %s", exc)
            raise
