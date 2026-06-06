"""Module for evaluating pose quality metrics from localization results.

Computes aggregate statistics such as OK rate, fallback rate, rejection rate,
mean residual RMSE, and drift warnings — all without requiring ground truth.

Physical warning codes:
    #P2-PREDICT-015: Drift warning when consecutive fallback exceeds threshold.
    #P2-PREDICT-016: OK/Fallback ratio warning when fallback rate is too high.
"""

import csv
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

logger = logging.getLogger("pose_evaluator")


@dataclass(frozen=True)
class PoseEvalMetrics:
    """Aggregated pose quality metrics for a localization run.

    Args:
        num_frames: Total number of frames processed.
        num_ok: Frames with SVD localization success (not fallback).
        num_fallback: Frames where fallback pose was used.
        num_rejected: Frames with no pose output at all.
        ok_rate: Fraction of OK frames (0.0–1.0).
        fallback_rate: Fraction of fallback frames (0.0–1.0).
        rejection_rate: Fraction of rejected frames (0.0–1.0).
        mean_residual_rmse: Mean RMSE over OK frames (None if no OK frames).
        max_residual_rmse: Worst-case RMSE over OK frames (None if no OK frames).
        max_consecutive_fallback: Longest consecutive fallback streak.
        drift_warning: True if max_consecutive_fallback exceeds drift threshold (#P2-PREDICT-015).
        high_fallback_ratio_warning: True if fallback_rate exceeds ratio threshold (#P2-PREDICT-016).
        warnings: List of human-readable warning strings.
    """

    num_frames: int
    num_ok: int
    num_fallback: int
    num_rejected: int
    ok_rate: float
    fallback_rate: float
    rejection_rate: float
    mean_residual_rmse: Optional[float]
    max_residual_rmse: Optional[float]
    max_consecutive_fallback: int
    drift_warning: bool
    high_fallback_ratio_warning: bool
    warnings: List[str] = field(default_factory=list)


def _safe_float(val):
    """Return float or None for empty/invalid values."""
    if val is None or val == "":
        return None
    try:
        return float(val)
    except (TypeError, ValueError):
        return None


def evaluate_poses_from_csv(
    poses_csv_path: Union[str, Path],
    drift_threshold: int = 5,
    fallback_ratio_threshold: float = 0.40,
) -> PoseEvalMetrics:
    """Compute pose quality metrics from a poses.csv file.

    Args:
        poses_csv_path: Path to poses.csv output from LocalizationWriter.
        drift_threshold: Maximum consecutive fallback frames before
            triggering drift warning (#P2-PREDICT-015). Default: 5.
        fallback_ratio_threshold: Fallback rate above which high fallback
            ratio warning (#P2-PREDICT-016) is triggered. Default: 0.40.

    Returns:
        PoseEvalMetrics with all computed statistics.

    Raises:
        FileNotFoundError: If poses_csv_path does not exist.
        ValueError: If CSV is malformed or missing required columns.
    """
    path = Path(poses_csv_path)
    if not path.exists():
        raise FileNotFoundError(f"poses.csv not found: {path}")

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    required_cols = {"frame_index", "status", "is_fallback", "residual_rmse"}
    if rows:
        actual_cols = set(rows[0].keys())
        missing = required_cols - actual_cols
        if missing:
            raise ValueError(f"poses.csv missing required columns: {missing}")

    num_frames = len(rows)

    if num_frames == 0:
        return PoseEvalMetrics(
            num_frames=0,
            num_ok=0,
            num_fallback=0,
            num_rejected=0,
            ok_rate=0.0,
            fallback_rate=0.0,
            rejection_rate=0.0,
            mean_residual_rmse=None,
            max_residual_rmse=None,
            max_consecutive_fallback=0,
            drift_warning=False,
            high_fallback_ratio_warning=False,
            warnings=["No frames in poses.csv"],
        )

    num_ok = 0
    num_fallback = 0
    num_rejected = 0
    ok_rmse_values = []
    max_consecutive_fb = 0
    current_consecutive_fb = 0

    for row in rows:
        status = row.get("status", "")
        is_fb = row.get("is_fallback", "false").strip().lower() == "true"

        if status == "OK" and not is_fb:
            num_ok += 1
            current_consecutive_fb = 0
            rmse = _safe_float(row.get("residual_rmse", ""))
            if rmse is not None:
                ok_rmse_values.append(rmse)
        elif is_fb:
            num_fallback += 1
            current_consecutive_fb += 1
            max_consecutive_fb = max(max_consecutive_fb, current_consecutive_fb)
        else:
            num_rejected += 1
            current_consecutive_fb += 1
            max_consecutive_fb = max(max_consecutive_fb, current_consecutive_fb)

    ok_rate = num_ok / num_frames if num_frames > 0 else 0.0
    fallback_rate = num_fallback / num_frames if num_frames > 0 else 0.0
    rejection_rate = num_rejected / num_frames if num_frames > 0 else 0.0

    mean_rmse = None
    max_rmse = None
    if ok_rmse_values:
        mean_rmse = sum(ok_rmse_values) / len(ok_rmse_values)
        max_rmse = max(ok_rmse_values)

    # Drift warning (#P2-PREDICT-015)
    drift_warn = max_consecutive_fb > drift_threshold

    # High fallback ratio warning (#P2-PREDICT-016)
    high_fb_warn = fallback_rate > fallback_ratio_threshold

    # Build warnings list
    warnings = []
    if drift_warn:
        warnings.append(
            f"#P2-PREDICT-015: Drift warning — {max_consecutive_fb} consecutive "
            f"fallback frames exceeds threshold ({drift_threshold}). "
            f"Robot may have drifted significantly from true position."
        )
    if high_fb_warn:
        warnings.append(
            f"#P2-PREDICT-016: High fallback ratio — {fallback_rate:.1%} of frames "
            f"used fallback (threshold: {fallback_ratio_threshold:.0%}). "
            f"Check RF map density or detection quality."
        )
    if num_ok == 0 and num_frames > 0:
        warnings.append(
            "CRITICAL: Zero OK frames — SVD localization never succeeded. "
            "Check RF map, detection quality, and config thresholds."
        )
    if max_rmse is not None and max_rmse > 0.10:
        warnings.append(
            f"HIGH_RESIDUAL: Worst-case RMSE = {max_rmse:.4f}m exceeds 0.10m. "
            f"Possible map coordinate error or noisy detections."
        )

    logger.info(
        "Pose evaluation: %d frames, OK=%d (%.1f%%), Fallback=%d (%.1f%%), "
        "Rejected=%d (%.1f%%), max_consec_fb=%d",
        num_frames, num_ok, ok_rate * 100, num_fallback, fallback_rate * 100,
        num_rejected, rejection_rate * 100, max_consecutive_fb,
    )

    return PoseEvalMetrics(
        num_frames=num_frames,
        num_ok=num_ok,
        num_fallback=num_fallback,
        num_rejected=num_rejected,
        ok_rate=ok_rate,
        fallback_rate=fallback_rate,
        rejection_rate=rejection_rate,
        mean_residual_rmse=mean_rmse,
        max_residual_rmse=max_rmse,
        max_consecutive_fallback=max_consecutive_fb,
        drift_warning=drift_warn,
        high_fallback_ratio_warning=high_fb_warn,
        warnings=warnings,
    )


def evaluate_poses_from_results(
    results_dir: Union[str, Path],
    drift_threshold: int = 5,
    fallback_ratio_threshold: float = 0.40,
) -> PoseEvalMetrics:
    """Convenience wrapper: evaluate from a results directory containing poses.csv.

    Args:
        results_dir: Directory containing poses.csv.
        drift_threshold: Drift warning threshold for consecutive fallback.
        fallback_ratio_threshold: Threshold for high fallback ratio warning.

    Returns:
        PoseEvalMetrics with all computed statistics.
    """
    path = Path(results_dir) / "poses.csv"
    return evaluate_poses_from_csv(path, drift_threshold, fallback_ratio_threshold)
