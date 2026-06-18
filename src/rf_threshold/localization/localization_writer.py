"""Module for writing localization results and trace diagnostics to CSV and JSON formats with fallback support."""

import csv
import json
import logging
import math
from pathlib import Path
from typing import Any, Dict, List, Union

from rf_threshold.localization.pose import LocalizationResult, LocalizationStatus
from rf_threshold.localization.fallback_manager import FallbackOutput

logger = logging.getLogger("localization_writer")


class LocalizationWriter:
    """Accumulates and writes SVD-based localization results, fallback states, and diagnostic traces."""

    def __init__(self, output_dir: Union[str, Path]) -> None:
        """Initialize the localization writer.

        Args:
            output_dir: Directory where results and debug files should be saved.
        """
        self.output_dir = Path(output_dir)

    def write_results(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
        frame_indices: List[int],
        num_detections_list: List[int],
    ) -> None:
        """Write all 8 outputs to the output directory.

        Args:
            results: List of LocalizationResult objects per frame.
            fallback_outputs: List of corresponding FallbackOutput objects.
            frame_indices: Corresponding frame indices.
            num_detections_list: Number of initial detections observed.

        Raises:
            ValueError: If the input lists have mismatching lengths.
        """
        if (
            len(results) != len(frame_indices)
            or len(results) != len(num_detections_list)
            or len(results) != len(fallback_outputs)
        ):
            raise ValueError(
                "Mismatching list lengths: "
                f"results={len(results)}, "
                f"fallback_outputs={len(fallback_outputs)}, "
                f"frame_indices={len(frame_indices)}, "
                f"num_detections_list={len(num_detections_list)}"
            )

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._write_poses_csv(results, fallback_outputs, frame_indices)
        self._write_poses_json(results, fallback_outputs, frame_indices)
        self._write_rejected_frames_csv(results, fallback_outputs, frame_indices, num_detections_list)
        self._write_association_debug_csv(results, fallback_outputs, frame_indices)
        self._write_svd_debug_csv(results, fallback_outputs, frame_indices)
        self._write_geometry_debug_csv(results, frame_indices)
        self._write_frame_debug_csv(results, fallback_outputs, frame_indices, num_detections_list)
        self._write_summary_csv(results, fallback_outputs)

        logger.info("Successfully wrote all 8 localization debug files with fallback to %s", self.output_dir)

    def _write_poses_csv(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
        frame_indices: List[int],
    ) -> None:
        """Write poses.csv file."""
        csv_path = self.output_dir / "poses.csv"
        try:
            with csv_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "frame_index",
                        "stamp",
                        "status",
                        "x",
                        "y",
                        "yaw",
                        "num_matches",
                        "residual_rmse",
                        "max_residual",
                        "is_fallback",
                        "fallback_source",
                        "consecutive_fallback_count",
                        "reason",
                    ]
                )
                for res, out, idx in zip(results, fallback_outputs, frame_indices):
                    p = out.pose
                    assoc_check = res.debug_info.get("assoc_check", {}) if res.debug_info else {}
                    max_res = assoc_check.get("best_max_residual", "")

                    is_fallback_str = "true" if out.is_fallback else "false"
                    fallback_source_str = out.fallback_source if out.fallback_source is not None else ""

                    if p is not None:
                        writer.writerow(
                            [
                                idx,
                                f"{res.stamp:.6f}",
                                out.status.value,
                                f"{p.x:.4f}",
                                f"{p.y:.4f}",
                                f"{p.yaw:.4f}",
                                p.num_matches,
                                f"{p.residual_rmse:.4f}",
                                f"{max_res:.4f}" if isinstance(max_res, (int, float)) else max_res,
                                is_fallback_str,
                                fallback_source_str,
                                out.consecutive_fallback_count,
                                res.reason if not out.is_fallback else f"Fallback applied: {res.reason}",
                            ]
                        )
                    else:
                        writer.writerow(
                            [
                                idx,
                                f"{res.stamp:.6f}",
                                out.status.value,
                                "",
                                "",
                                "",
                                0,
                                "",
                                "",
                                is_fallback_str,
                                fallback_source_str,
                                out.consecutive_fallback_count,
                                res.reason,
                            ]
                        )
        except Exception as exc:
            logger.error("Failed to write poses.csv: %s", exc)
            raise

    def _write_poses_json(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
        frame_indices: List[int],
    ) -> None:
        """Write poses.json file."""
        json_path = self.output_dir / "poses.json"
        poses_list = []
        for res, out, idx in zip(results, fallback_outputs, frame_indices):
            p = out.pose
            pose_dict: Dict[str, Any] = {
                "frame_index": idx,
                "stamp": float(res.stamp),
                "status": out.status.value,
                "is_fallback": out.is_fallback,
                "fallback_source": out.fallback_source,
                "consecutive_fallback_count": out.consecutive_fallback_count,
                "reason": res.reason if not out.is_fallback else f"Fallback applied: {res.reason}",
            }
            if p is not None:
                pose_dict.update(
                    {
                        "x": float(p.x),
                        "y": float(p.y),
                        "yaw": float(p.yaw),
                        "num_matches": int(p.num_matches),
                        "residual_rmse": float(p.residual_rmse),
                    }
                )
            poses_list.append(pose_dict)

        try:
            with json_path.open("w", encoding="utf-8") as file:
                json.dump({"poses": poses_list}, file, indent=2)
        except Exception as exc:
            logger.error("Failed to write poses.json: %s", exc)
            raise

    def _write_rejected_frames_csv(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
        frame_indices: List[int],
        num_detections_list: List[int],
    ) -> None:
        """Write rejected_frames.csv file."""
        csv_path = self.output_dir / "rejected_frames.csv"
        try:
            with csv_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "frame_index",
                        "stamp",
                        "original_status",
                        "reason",
                        "num_detections",
                        "num_matches",
                        "fallback_used",
                        "fallback_status",
                    ]
                )
                for res, out, idx, n_det in zip(results, fallback_outputs, frame_indices, num_detections_list):
                    if res.status != LocalizationStatus.OK:
                        fb_used_str = "true" if out.is_fallback else "false"
                        fb_status_str = out.status.value if out.is_fallback else ""
                        writer.writerow(
                            [
                                idx,
                                f"{res.stamp:.6f}",
                                res.status.value,
                                res.reason,
                                n_det,
                                len(res.matched_pairs),
                                fb_used_str,
                                fb_status_str,
                            ]
                        )
        except Exception as exc:
            logger.error("Failed to write rejected_frames.csv: %s", exc)
            raise

    def _write_association_debug_csv(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
        frame_indices: List[int],
    ) -> None:
        """Write association_debug.csv file."""
        csv_path = self.output_dir / "association_debug.csv"
        try:
            with csv_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "frame_index",
                        "stamp",
                        "detection_id",
                        "landmark_id",
                        "x_lidar",
                        "y_lidar",
                        "x_map",
                        "y_map",
                        "weight",
                        "residual",
                    ]
                )
                for res, out, idx in zip(results, fallback_outputs, frame_indices):
                    p = out.pose
                    has_pose = p is not None
                    c = math.cos(p.yaw) if has_pose else 1.0
                    s = math.sin(p.yaw) if has_pose else 0.0

                    for pair in res.matched_pairs:
                        resid_str = ""
                        if has_pose and p is not None:
                            # points_map ≈ R * points_lidar + t
                            rx = c * pair.point_lidar[0] - s * pair.point_lidar[1] + p.x
                            ry = s * pair.point_lidar[0] + c * pair.point_lidar[1] + p.y
                            resid = math.sqrt((pair.point_map[0] - rx) ** 2 + (pair.point_map[1] - ry) ** 2)
                            resid_str = f"{resid:.4f}"

                        writer.writerow(
                            [
                                idx,
                                f"{res.stamp:.6f}",
                                pair.detection_id,
                                pair.landmark_id,
                                f"{pair.point_lidar[0]:.4f}",
                                f"{pair.point_lidar[1]:.4f}",
                                f"{pair.point_map[0]:.4f}",
                                f"{pair.point_map[1]:.4f}",
                                f"{pair.weight:.4f}",
                                resid_str,
                            ]
                        )
        except Exception as exc:
            logger.error("Failed to write association_debug.csv: %s", exc)
            raise

    def _write_svd_debug_csv(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
        frame_indices: List[int],
    ) -> None:
        """Write svd_debug.csv file."""
        csv_path = self.output_dir / "svd_debug.csv"
        try:
            with csv_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "frame_index",
                        "stamp",
                        "R00",
                        "R01",
                        "R10",
                        "R11",
                        "tx",
                        "ty",
                        "yaw",
                        "det_R",
                        "residual_rmse",
                        "max_residual",
                        "num_points",
                    ]
                )
                for res, out, idx in zip(results, fallback_outputs, frame_indices):
                    p = out.pose
                    assoc_check = res.debug_info.get("assoc_check", {}) if res.debug_info else {}
                    max_res = assoc_check.get("best_max_residual", "")

                    if p is not None:
                        c = math.cos(p.yaw)
                        s = math.sin(p.yaw)
                        det_R = c * c + s * s
                        writer.writerow(
                            [
                                idx,
                                f"{res.stamp:.6f}",
                                f"{c:.6f}",
                                f"{-s:.6f}",
                                f"{s:.6f}",
                                f"{c:.6f}",
                                f"{p.x:.4f}",
                                f"{p.y:.4f}",
                                f"{p.yaw:.4f}",
                                f"{det_R:.4f}",
                                f"{p.residual_rmse:.4f}",
                                f"{max_res:.4f}" if isinstance(max_res, (int, float)) else max_res,
                                p.num_matches,
                            ]
                        )
                    else:
                        writer.writerow(
                            [
                                idx,
                                f"{res.stamp:.6f}",
                                "",
                                "",
                                "",
                                "",
                                "",
                                "",
                                "",
                                "",
                                f"{res.residual_rmse:.4f}" if res.residual_rmse is not None else "",
                                f"{max_res:.4f}" if isinstance(max_res, (int, float)) else max_res,
                                len(res.matched_pairs),
                            ]
                        )
        except Exception as exc:
            logger.error("Failed to write svd_debug.csv: %s", exc)
            raise

    def _write_geometry_debug_csv(
        self, results: List[LocalizationResult], frame_indices: List[int]
    ) -> None:
        """Write geometry_debug.csv file."""
        csv_path = self.output_dir / "geometry_debug.csv"
        try:
            with csv_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "frame_index",
                        "stamp",
                        "num_pairs",
                        "spread_lidar",
                        "spread_map",
                        "condition_number_lidar",
                        "condition_number_map",
                        "warning",
                        "is_valid",
                        "is_degenerate",
                    ]
                )
                for res, idx in zip(results, frame_indices):
                    geom_check = res.debug_info.get("geom_check", {}) if res.debug_info else {}
                    cond_lidar = geom_check.get("condition_number_lidar", "")
                    cond_map = geom_check.get("condition_number_map", "")
                    spread_lidar = geom_check.get("spread_lidar", "")
                    spread_map = geom_check.get("spread_map", "")

                    # Attempt to extract geometry outcome
                    warning_val = ""
                    is_valid_str = "false"
                    is_degenerate_str = "false"

                    if res.status == LocalizationStatus.OK:
                        is_valid_str = "true"
                    elif res.status == LocalizationStatus.DEGENERATE_GEOMETRY:
                        is_valid_str = "false"
                        is_degenerate_str = "true"

                    if "collinear" in res.reason.lower() or "near-collinear" in res.reason.lower():
                        warning_val = "NEAR_COLLINEAR"
                        is_degenerate_str = "true"

                    writer.writerow(
                        [
                            idx,
                            f"{res.stamp:.6f}",
                            len(res.matched_pairs),
                            f"{spread_lidar:.4f}" if isinstance(spread_lidar, (int, float)) else spread_lidar,
                            f"{spread_map:.4f}" if isinstance(spread_map, (int, float)) else spread_map,
                            f"{cond_lidar:.4f}" if isinstance(cond_lidar, (int, float)) else cond_lidar,
                            f"{cond_map:.4f}" if isinstance(cond_map, (int, float)) else cond_map,
                            warning_val,
                            is_valid_str,
                            is_degenerate_str,
                        ]
                    )
        except Exception as exc:
            logger.error("Failed to write geometry_debug.csv: %s", exc)
            raise

    def _write_frame_debug_csv(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
        frame_indices: List[int],
        num_detections_list: List[int],
    ) -> None:
        """Write frame_debug.csv file."""
        csv_path = self.output_dir / "frame_debug.csv"
        try:
            with csv_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "frame_index",
                        "stamp",
                        "num_detections",
                        "num_filtered_detections",
                        "num_matches",
                        "status",
                        "is_fallback",
                        "reason",
                    ]
                )
                for res, out, idx, n_det in zip(results, fallback_outputs, frame_indices, num_detections_list):
                    # We treat all detections as filtered for simplicity
                    writer.writerow(
                        [
                            idx,
                            f"{res.stamp:.6f}",
                            n_det,
                            n_det,
                            len(res.matched_pairs),
                            out.status.value,
                            "true" if out.is_fallback else "false",
                            res.reason if not out.is_fallback else f"Fallback applied: {res.reason}",
                        ]
                    )
        except Exception as exc:
            logger.error("Failed to write frame_debug.csv: %s", exc)
            raise

    def _write_summary_csv(
        self,
        results: List[LocalizationResult],
        fallback_outputs: List[FallbackOutput],
    ) -> None:
        """Write summary as key-value metric format in localization_summary.csv."""
        csv_path = self.output_dir / "localization_summary.csv"

        num_frames = len(results)
        num_ok = 0
        num_fallback = 0
        num_rejected_without_fallback = 0

        # Categorized original failure causes
        num_insufficient_dets = 0
        num_assoc_failed = 0
        num_degenerate_geom = 0
        num_high_res = 0

        for res, out in zip(results, fallback_outputs):
            if out.status == LocalizationStatus.OK:
                num_ok += 1
            elif out.is_fallback:
                num_fallback += 1
            else:
                num_rejected_without_fallback += 1

            # Count the localization failures
            if res.status == LocalizationStatus.INSUFFICIENT_DETECTIONS:
                num_insufficient_dets += 1
            elif res.status == LocalizationStatus.ASSOCIATION_FAILED:
                num_assoc_failed += 1
            elif res.status == LocalizationStatus.DEGENERATE_GEOMETRY:
                num_degenerate_geom += 1
            elif res.status == LocalizationStatus.HIGH_RESIDUAL:
                num_high_res += 1

        try:
            with csv_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["metric", "value"])
                writer.writerow(["num_frames", num_frames])
                writer.writerow(["num_ok", num_ok])
                writer.writerow(["num_fallback", num_fallback])
                writer.writerow(["num_rejected_without_fallback", num_rejected_without_fallback])
                writer.writerow(["num_insufficient_detections", num_insufficient_dets])
                writer.writerow(["num_association_failed", num_assoc_failed])
                writer.writerow(["num_degenerate_geometry", num_degenerate_geom])
                writer.writerow(["num_high_residual", num_high_res])
        except Exception as exc:
            logger.error("Failed to write localization_summary.csv: %s", exc)
            raise
