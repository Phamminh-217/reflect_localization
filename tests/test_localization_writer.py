"""Unit tests for the LocalizationWriter class covering all 7 SRE/QA scenarios with fallback."""

import csv
import json
import math
from pathlib import Path
import pytest
import numpy as np

from rf_threshold.localization.pose import (
    LocalizationResult,
    LocalizationStatus,
    MatchedPair,
    RobotPose,
)
from rf_threshold.localization.fallback_manager import FallbackOutput
from rf_threshold.localization.localization_writer import LocalizationWriter


@pytest.fixture
def sample_results() -> tuple:
    """Create sample mixed results and fallback states (success, fallback, hard rejection)."""
    # 1. OK Frame
    pose0 = RobotPose(
        stamp=1716000000.100,
        frame_id="map_frame",
        child_frame_id="lidar_frame",
        x=1.24,
        y=0.35,
        yaw=0.02,
        residual_rmse=0.015,
        num_matches=3,
    )
    res0 = LocalizationResult(
        stamp=1716000000.100,
        status=LocalizationStatus.OK,
        pose=pose0,
        matched_pairs=[
            MatchedPair(0, 10, np.array([1.0, 2.0, 0.0]), np.array([1.1, 2.05, 0.0]), 1.0),
            MatchedPair(1, 11, np.array([2.0, 3.0, 0.0]), np.array([2.1, 3.05, 0.0]), 1.0),
            MatchedPair(2, 12, np.array([3.0, 4.0, 0.0]), np.array([3.1, 4.05, 0.0]), 1.0),
        ],
        residual_rmse=0.015,
        reason="Localization succeeded.",
        debug_info={
            "geom_check": {
                "condition_number_lidar": 12.5,
                "condition_number_map": 12.8,
                "spread_lidar": 1.45,
                "spread_map": 1.48,
            },
            "assoc_check": {
                "num_observed_triplets": 10,
                "num_map_triplets": 20,
                "num_triplet_candidates": 5,
                "num_svd_candidates": 2,
                "best_num_inliers": 3,
                "best_residual_rmse": 0.015,
                "best_max_residual": 0.024,
            },
        },
    )
    out0 = FallbackOutput(
        status=LocalizationStatus.OK,
        pose=pose0,
        is_fallback=False,
        fallback_source=None,
        consecutive_fallback_count=0,
    )

    # 2. Fallback Frame (INSUFFICIENT_DETECTIONS, but has fallback pose)
    pose1 = RobotPose(
        stamp=1716000000.200,
        frame_id="map_frame",
        child_frame_id="lidar_frame",
        x=1.24,
        y=0.35,
        yaw=0.02,
        residual_rmse=0.015,
        num_matches=3,
    )
    res1 = LocalizationResult(
        stamp=1716000000.200,
        status=LocalizationStatus.INSUFFICIENT_DETECTIONS,
        pose=None,
        matched_pairs=[],
        residual_rmse=None,
        reason="Insufficient detections: 2 < 3",
        debug_info={"num_detections": 2},
    )
    out1 = FallbackOutput(
        status=LocalizationStatus.FALLBACK_LAST_VALID_POSE,
        pose=pose1,
        is_fallback=True,
        fallback_source="last_valid_pose",
        consecutive_fallback_count=1,
    )

    # 3. Failed Frame (DEGENERATE_GEOMETRY, no fallback)
    res2 = LocalizationResult(
        stamp=1716000000.300,
        status=LocalizationStatus.DEGENERATE_GEOMETRY,
        pose=None,
        matched_pairs=[
            MatchedPair(0, 10, np.array([1.0, 1.0, 0.0]), np.array([1.1, 1.1, 0.0]), 1.0),
            MatchedPair(1, 10, np.array([1.0, 1.0, 0.0]), np.array([1.1, 1.1, 0.0]), 1.0),
        ],
        residual_rmse=0.045,
        reason="Duplicate detection or landmark ID detected.",
        debug_info={
            "geom_check": {
                "condition_number_lidar": float("inf"),
                "condition_number_map": float("inf"),
                "spread_lidar": 0.0,
                "spread_map": 0.0,
            }
        },
    )
    out2 = FallbackOutput(
        status=LocalizationStatus.DEGENERATE_GEOMETRY,
        pose=None,
        is_fallback=False,
        fallback_source=None,
        consecutive_fallback_count=2,
    )

    results = [res0, res1, res2]
    fallback_outputs = [out0, out1, out2]
    frame_indices = [100, 101, 102]
    num_detections = [4, 2, 3]

    return results, fallback_outputs, frame_indices, num_detections


def test_writer_poses_csv(tmp_path: Path, sample_results: tuple) -> None:
    """Scenario 1: Verify poses.csv has correct fallback headers and columns for OK/fallback/fail frames."""
    results, fallback_outputs, indices, num_detections = sample_results
    writer = LocalizationWriter(tmp_path)
    writer.write_results(results, fallback_outputs, indices, num_detections)

    csv_file = tmp_path / "poses.csv"
    assert csv_file.exists()

    with csv_file.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)

    assert len(rows) == 4  # Header + 3 frames
    assert rows[0] == [
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

    # OK Frame
    assert rows[1][0] == "100"
    assert rows[1][2] == "OK"
    assert rows[1][3] == "1.2400"
    assert rows[1][9] == "false"
    assert rows[1][10] == ""
    assert rows[1][11] == "0"

    # Fallback Frame
    assert rows[2][0] == "101"
    assert rows[2][2] == "FALLBACK_LAST_VALID_POSE"
    assert rows[2][3] == "1.2400"
    assert rows[2][9] == "true"
    assert rows[2][10] == "last_valid_pose"
    assert rows[2][11] == "1"


def test_writer_poses_json(tmp_path: Path, sample_results: tuple) -> None:
    """Verify poses.json contains all parsed pose metadata and is_fallback attributes."""
    results, fallback_outputs, indices, num_detections = sample_results
    writer = LocalizationWriter(tmp_path)
    writer.write_results(results, fallback_outputs, indices, num_detections)

    json_file = tmp_path / "poses.json"
    assert json_file.exists()

    with json_file.open("r", encoding="utf-8") as f:
        data = json.load(f)

    assert "poses" in data
    assert len(data["poses"]) == 3
    p1 = data["poses"][1]
    assert p1["frame_index"] == 101
    assert p1["status"] == "FALLBACK_LAST_VALID_POSE"
    assert p1["is_fallback"] is True
    assert p1["fallback_source"] == "last_valid_pose"
    assert p1["x"] == pytest.approx(1.24)


def test_writer_rejected_frames_csv(tmp_path: Path, sample_results: tuple) -> None:
    """Scenario 2: Verify rejected_frames.csv outputs only failed frames with fallback flags."""
    results, fallback_outputs, indices, num_detections = sample_results
    writer = LocalizationWriter(tmp_path)
    writer.write_results(results, fallback_outputs, indices, num_detections)

    csv_file = tmp_path / "rejected_frames.csv"
    assert csv_file.exists()

    with csv_file.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)

    assert len(rows) == 3  # Header + 2 failed SVD frames
    assert rows[0] == [
        "frame_index",
        "stamp",
        "original_status",
        "reason",
        "num_detections",
        "num_matches",
        "fallback_used",
        "fallback_status",
    ]

    # Frame 101 (failed SVD but used fallback)
    assert rows[1][0] == "101"
    assert rows[1][2] == "INSUFFICIENT_DETECTIONS"
    assert rows[1][6] == "true"
    assert rows[1][7] == "FALLBACK_LAST_VALID_POSE"

    # Frame 102 (failed SVD, no fallback)
    assert rows[2][0] == "102"
    assert rows[2][2] == "DEGENERATE_GEOMETRY"
    assert rows[2][6] == "false"
    assert rows[2][7] == ""


def test_writer_association_debug_csv(tmp_path: Path, sample_results: tuple) -> None:
    """Scenario 3: Verify association_debug.csv has correctly computed on-the-fly residuals."""
    results, fallback_outputs, indices, num_detections = sample_results
    writer = LocalizationWriter(tmp_path)
    writer.write_results(results, fallback_outputs, indices, num_detections)

    csv_file = tmp_path / "association_debug.csv"
    assert csv_file.exists()

    with csv_file.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)

    assert len(rows) == 6
    assert rows[0] == [
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

    # Frame 100 first match residual verification
    # center = [1.24, 0.35], yaw = 0.02
    # lidar = [1.0, 2.0], map = [1.1, 2.05]
    # Rx = cos(0.02)*1.0 - sin(0.02)*2.0 + 1.24
    # Ry = sin(0.02)*1.0 + cos(0.02)*2.0 + 0.35
    c = math.cos(0.02)
    s = math.sin(0.02)
    rx = c * 1.0 - s * 2.0 + 1.24
    ry = s * 1.0 + c * 2.0 + 0.35
    resid = math.sqrt((1.1 - rx) ** 2 + (2.05 - ry) ** 2)

    assert rows[1][0] == "100"
    assert float(rows[1][9]) == pytest.approx(resid, abs=1e-4)


def test_writer_svd_debug_csv(tmp_path: Path, sample_results: tuple) -> None:
    """Scenario 4: Verify reconstructed R matrices and det_R elements in svd_debug.csv."""
    results, fallback_outputs, indices, num_detections = sample_results
    writer = LocalizationWriter(tmp_path)
    writer.write_results(results, fallback_outputs, indices, num_detections)

    csv_file = tmp_path / "svd_debug.csv"
    assert csv_file.exists()

    with csv_file.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)

    assert len(rows) == 4
    assert rows[0] == [
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

    # OK Frame R determinant check
    assert rows[1][0] == "100"
    assert float(rows[1][9]) == pytest.approx(1.0)
    assert rows[1][11] == "0.0240"


def test_writer_geometry_debug_csv(tmp_path: Path, sample_results: tuple) -> None:
    """Scenario 5: Verify condition numbers and is_degenerate fields in geometry_debug.csv."""
    results, fallback_outputs, indices, num_detections = sample_results
    writer = LocalizationWriter(tmp_path)
    writer.write_results(results, fallback_outputs, indices, num_detections)

    csv_file = tmp_path / "geometry_debug.csv"
    assert csv_file.exists()

    with csv_file.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)

    assert len(rows) == 4
    assert rows[0] == [
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

    assert rows[1][0] == "100"
    assert rows[1][2] == "3"
    assert rows[1][8] == "true"
    assert rows[1][9] == "false"

    assert rows[3][0] == "102"
    assert rows[3][2] == "2"
    assert rows[3][8] == "false"
    assert rows[3][9] == "true"


def test_writer_summary_csv(tmp_path: Path, sample_results: tuple) -> None:
    """Verify localization_summary.csv outputs metrics separating OK and fallback frames."""
    results, fallback_outputs, indices, num_detections = sample_results
    writer = LocalizationWriter(tmp_path)
    writer.write_results(results, fallback_outputs, indices, num_detections)

    csv_file = tmp_path / "localization_summary.csv"
    assert csv_file.exists()

    summary = {}
    with csv_file.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            summary[row[0]] = int(row[1])

    assert summary["num_frames"] == 3
    assert summary["num_ok"] == 1
    assert summary["num_fallback"] == 1
    assert summary["num_rejected_without_fallback"] == 1
    assert summary["num_insufficient_detections"] == 1
    assert summary["num_degenerate_geometry"] == 1


def test_writer_robustness_empty_list(tmp_path: Path) -> None:
    """Scenario 7: Ensure writer handles empty fallback lists cleanly and does not crash."""
    writer = LocalizationWriter(tmp_path)
    writer.write_results([], [], [], [])

    # Files should still contain the headers
    assert (tmp_path / "poses.csv").exists()
    assert (tmp_path / "rejected_frames.csv").exists()
    assert (tmp_path / "localization_summary.csv").exists()
