"""Integration tests for Phase 2.6.4 — plot_localization_debug.py CLI script.

Tests verify that the script:
  1. Runs without error on synthetic CSV outputs from LocalizationWriter.
  2. Saves the expected PNG plots when --save is specified.
  3. Exits gracefully when required files are missing.
  4. Handles frames with no pose (pure rejected frames) without crashing.
"""

import csv
import json
import subprocess
import sys
from pathlib import Path
from typing import Dict, List

import pytest


# --------------------------------------------------------------------------- #
# Helpers                                                                       #
# --------------------------------------------------------------------------- #

def _write_csv(path: Path, headers: List[str], rows: List[Dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


# --------------------------------------------------------------------------- #
# Fixture: synthetic localization output directory                              #
# --------------------------------------------------------------------------- #

@pytest.fixture
def synthetic_output(tmp_path: Path) -> Path:
    """Create a minimal but complete set of 8 localization debug files."""
    out = tmp_path / "loc_out"
    out.mkdir()

    # poses.csv
    _write_csv(
        out / "poses.csv",
        headers=[
            "frame_index", "stamp", "status", "x", "y", "yaw",
            "num_matches", "residual_rmse", "max_residual",
            "is_fallback", "fallback_source", "consecutive_fallback_count", "reason",
        ],
        rows=[
            {
                "frame_index": 0, "stamp": "100.000000", "status": "OK",
                "x": "1.0", "y": "2.0", "yaw": "0.1",
                "num_matches": 3, "residual_rmse": "0.01", "max_residual": "0.02",
                "is_fallback": "false", "fallback_source": "",
                "consecutive_fallback_count": 0, "reason": "Localization OK",
            },
            {
                "frame_index": 1, "stamp": "101.000000", "status": "FALLBACK_LAST_VALID_POSE",
                "x": "1.0", "y": "2.0", "yaw": "0.1",
                "num_matches": 0, "residual_rmse": "0.00", "max_residual": "",
                "is_fallback": "true", "fallback_source": "frame_0",
                "consecutive_fallback_count": 1,
                "reason": "Fallback applied: INSUFFICIENT_DETECTIONS",
            },
            {
                "frame_index": 2, "stamp": "102.000000",
                "status": "INSUFFICIENT_DETECTIONS",
                "x": "", "y": "", "yaw": "",
                "num_matches": 0, "residual_rmse": "", "max_residual": "",
                "is_fallback": "false", "fallback_source": "",
                "consecutive_fallback_count": 2,
                "reason": "Insufficient detections: 2 < 3",
            },
        ],
    )

    # rejected_frames.csv
    _write_csv(
        out / "rejected_frames.csv",
        headers=[
            "frame_index", "stamp", "original_status", "reason",
            "num_detections", "num_matches", "fallback_used", "fallback_status",
        ],
        rows=[
            {
                "frame_index": 1, "stamp": "101.000000",
                "original_status": "INSUFFICIENT_DETECTIONS",
                "reason": "Insufficient detections: 2 < 3",
                "num_detections": 2, "num_matches": 0,
                "fallback_used": "true", "fallback_status": "FALLBACK_LAST_VALID_POSE",
            },
            {
                "frame_index": 2, "stamp": "102.000000",
                "original_status": "INSUFFICIENT_DETECTIONS",
                "reason": "Insufficient detections: 2 < 3",
                "num_detections": 2, "num_matches": 0,
                "fallback_used": "false", "fallback_status": "",
            },
        ],
    )

    # localization_summary.csv
    _write_csv(
        out / "localization_summary.csv",
        headers=["metric", "value"],
        rows=[
            {"metric": "num_frames", "value": 3},
            {"metric": "num_ok", "value": 1},
            {"metric": "num_fallback", "value": 1},
            {"metric": "num_rejected_without_fallback", "value": 1},
            {"metric": "num_insufficient_detections", "value": 2},
            {"metric": "num_association_failed", "value": 0},
            {"metric": "num_degenerate_geometry", "value": 0},
            {"metric": "num_high_residual", "value": 0},
        ],
    )

    # association_debug.csv
    _write_csv(
        out / "association_debug.csv",
        headers=[
            "frame_index", "stamp", "detection_id", "landmark_id",
            "x_lidar", "y_lidar", "x_map", "y_map", "weight", "residual",
        ],
        rows=[
            {
                "frame_index": 0, "stamp": "100.000000",
                "detection_id": 0, "landmark_id": 0,
                "x_lidar": "0.0", "y_lidar": "0.0",
                "x_map": "0.0", "y_map": "0.0",
                "weight": "1.0", "residual": "0.001",
            },
            {
                "frame_index": 0, "stamp": "100.000000",
                "detection_id": 1, "landmark_id": 1,
                "x_lidar": "1.5", "y_lidar": "0.0",
                "x_map": "1.5", "y_map": "0.0",
                "weight": "1.0", "residual": "0.001",
            },
            {
                "frame_index": 0, "stamp": "100.000000",
                "detection_id": 2, "landmark_id": 2,
                "x_lidar": "0.0", "y_lidar": "2.0",
                "x_map": "0.0", "y_map": "2.0",
                "weight": "1.0", "residual": "0.001",
            },
        ],
    )

    # svd_debug.csv
    _write_csv(
        out / "svd_debug.csv",
        headers=[
            "frame_index", "stamp", "R00", "R01", "R10", "R11",
            "tx", "ty", "yaw", "det_R", "residual_rmse", "max_residual", "num_points",
        ],
        rows=[
            {
                "frame_index": 0, "stamp": "100.000000",
                "R00": "1.0", "R01": "0.0", "R10": "0.0", "R11": "1.0",
                "tx": "1.0", "ty": "2.0", "yaw": "0.1",
                "det_R": "1.0", "residual_rmse": "0.01", "max_residual": "0.02",
                "num_points": 3,
            },
            {
                "frame_index": 1, "stamp": "101.000000",
                "R00": "", "R01": "", "R10": "", "R11": "",
                "tx": "", "ty": "", "yaw": "",
                "det_R": "", "residual_rmse": "", "max_residual": "", "num_points": 0,
            },
            {
                "frame_index": 2, "stamp": "102.000000",
                "R00": "", "R01": "", "R10": "", "R11": "",
                "tx": "", "ty": "", "yaw": "",
                "det_R": "", "residual_rmse": "", "max_residual": "", "num_points": 0,
            },
        ],
    )

    # geometry_debug.csv
    _write_csv(
        out / "geometry_debug.csv",
        headers=[
            "frame_index", "stamp", "num_pairs",
            "spread_lidar", "spread_map",
            "condition_number_lidar", "condition_number_map",
            "warning", "is_valid", "is_degenerate",
        ],
        rows=[
            {
                "frame_index": 0, "stamp": "100.000000", "num_pairs": 3,
                "spread_lidar": "1.2", "spread_map": "1.2",
                "condition_number_lidar": "3.5", "condition_number_map": "3.5",
                "warning": "", "is_valid": "true", "is_degenerate": "false",
            },
            {
                "frame_index": 1, "stamp": "101.000000", "num_pairs": 0,
                "spread_lidar": "", "spread_map": "",
                "condition_number_lidar": "", "condition_number_map": "",
                "warning": "", "is_valid": "false", "is_degenerate": "false",
            },
            {
                "frame_index": 2, "stamp": "102.000000", "num_pairs": 0,
                "spread_lidar": "", "spread_map": "",
                "condition_number_lidar": "", "condition_number_map": "",
                "warning": "", "is_valid": "false", "is_degenerate": "false",
            },
        ],
    )

    # frame_debug.csv
    _write_csv(
        out / "frame_debug.csv",
        headers=[
            "frame_index", "stamp", "num_detections", "num_filtered_detections",
            "num_matches", "status", "is_fallback", "reason",
        ],
        rows=[
            {
                "frame_index": 0, "stamp": "100.000000",
                "num_detections": 3, "num_filtered_detections": 3,
                "num_matches": 3, "status": "OK",
                "is_fallback": "false", "reason": "Localization OK",
            },
            {
                "frame_index": 1, "stamp": "101.000000",
                "num_detections": 2, "num_filtered_detections": 2,
                "num_matches": 0, "status": "FALLBACK_LAST_VALID_POSE",
                "is_fallback": "true", "reason": "Fallback applied: Insufficient detections",
            },
            {
                "frame_index": 2, "stamp": "102.000000",
                "num_detections": 2, "num_filtered_detections": 2,
                "num_matches": 0, "status": "INSUFFICIENT_DETECTIONS",
                "is_fallback": "false", "reason": "Insufficient detections: 2 < 3",
            },
        ],
    )

    # poses.json (not directly used by plot script but part of the suite)
    poses_json = {
        "poses": [
            {
                "frame_index": 0, "stamp": 100.0, "status": "OK",
                "is_fallback": False, "fallback_source": None,
                "consecutive_fallback_count": 0, "reason": "Localization OK",
                "x": 1.0, "y": 2.0, "yaw": 0.1, "num_matches": 3, "residual_rmse": 0.01,
            },
        ]
    }
    with (out / "poses.json").open("w", encoding="utf-8") as f:
        json.dump(poses_json, f)

    return out


@pytest.fixture
def synthetic_map(tmp_path: Path) -> Path:
    """Write a minimal RF map JSON."""
    map_data = {
        "map_name": "test_map",
        "frame_id": "map_frame",
        "unit": "meter",
        "landmarks": [
            {"id": 0, "position_map": [0.0, 0.0, 0.0]},
            {"id": 1, "position_map": [1.5, 0.0, 0.0]},
            {"id": 2, "position_map": [0.0, 2.0, 0.0]},
        ],
    }
    map_file = tmp_path / "rf_map_v1.json"
    with map_file.open("w", encoding="utf-8") as f:
        json.dump(map_data, f)
    return map_file


# --------------------------------------------------------------------------- #
# Tests                                                                         #
# --------------------------------------------------------------------------- #

import os


def _run_plot_script(extra_args: List[str], env_extra: Dict = None):
    """Helper: run plot script via subprocess, return CompletedProcess."""
    env = os.environ.copy()
    env["PYTHONPATH"] = "src"
    # Use non-interactive matplotlib backend
    env["MPLBACKEND"] = "Agg"
    if env_extra:
        env.update(env_extra)
    cmd = [sys.executable, "scripts/plot_localization_debug.py"] + extra_args
    return subprocess.run(cmd, env=env, capture_output=True, text=True)


def test_plot_script_saves_all_plots(
    synthetic_output: Path, synthetic_map: Path, tmp_path: Path
) -> None:
    """Verify the script saves all 5 PNG plots when --save is specified."""
    save_dir = tmp_path / "plots"

    res = _run_plot_script(
        [
            "--output", str(synthetic_output),
            "--map", str(synthetic_map),
            "--frame", "0",
            "--save", str(save_dir),
        ]
    )
    assert res.returncode == 0, (
        f"Script failed with returncode {res.returncode}.\n"
        f"STDERR:\n{res.stderr}\n"
        f"STDOUT:\n{res.stdout}"
    )

    assert save_dir.exists(), "Plot save directory was not created."
    expected_plots = [
        "01_trajectory.png",
        "02_residuals.png",
        "03_geometry.png",
        "04_status_summary.png",
        "05_frame_0_association.png",
    ]
    for plot_name in expected_plots:
        assert (save_dir / plot_name).exists(), f"Missing expected plot: {plot_name}"


def test_plot_script_auto_selects_ok_frame(
    synthetic_output: Path, synthetic_map: Path, tmp_path: Path
) -> None:
    """Verify the script auto-selects first OK frame when --frame is not specified."""
    save_dir = tmp_path / "plots_auto"

    res = _run_plot_script(
        [
            "--output", str(synthetic_output),
            "--map", str(synthetic_map),
            "--save", str(save_dir),
        ]
    )
    assert res.returncode == 0, f"Script failed.\nSTDERR: {res.stderr}"
    # Frame 0 is the first OK frame -> should produce frame_0 association plot
    assert (save_dir / "05_frame_0_association.png").exists()


def test_plot_script_without_map(
    synthetic_output: Path, tmp_path: Path
) -> None:
    """Verify the script runs successfully even without a --map argument."""
    save_dir = tmp_path / "plots_nomap"

    res = _run_plot_script(
        [
            "--output", str(synthetic_output),
            "--frame", "0",
            "--save", str(save_dir),
        ]
    )
    assert res.returncode == 0, f"Script failed.\nSTDERR: {res.stderr}"
    assert (save_dir / "01_trajectory.png").exists()
    assert (save_dir / "05_frame_0_association.png").exists()


def test_plot_script_missing_output_dir(tmp_path: Path) -> None:
    """Verify the script exits with non-zero code when output dir is missing files."""
    missing_dir = tmp_path / "does_not_exist"
    missing_dir.mkdir()  # Exists but empty — missing required CSVs

    res = _run_plot_script(
        [
            "--output", str(missing_dir),
            "--save", str(tmp_path / "plots"),
        ]
    )
    assert res.returncode != 0, (
        "Expected non-zero exit code when required CSV files are missing."
    )


def test_plot_script_frame_with_no_pose(
    synthetic_output: Path, tmp_path: Path
) -> None:
    """Verify the script does not crash when --frame points to a rejected (no-pose) frame."""
    save_dir = tmp_path / "plots_rej"

    # Frame 2 has no pose (x/y/yaw are empty)
    res = _run_plot_script(
        [
            "--output", str(synthetic_output),
            "--frame", "2",
            "--save", str(save_dir),
        ]
    )
    # Should complete without crash (returncode 0), even though frame 2 has no association rows
    assert res.returncode == 0, f"Script crashed on rejected frame.\nSTDERR: {res.stderr}"
