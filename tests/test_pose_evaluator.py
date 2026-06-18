"""Unit tests for pose_evaluator module."""

import csv
import os
import pytest
from pathlib import Path

from rf_threshold.localization.pose_evaluator import (
    PoseEvalMetrics,
    evaluate_poses_from_csv,
    evaluate_poses_from_results,
)


@pytest.fixture
def tmp_dir(tmp_path):
    """Return a temporary directory for test artifacts."""
    return tmp_path


def _write_poses_csv(path, rows):
    """Helper to write a poses.csv file with given rows."""
    header = [
        "frame_index", "stamp", "status", "x", "y", "yaw",
        "num_matches", "residual_rmse", "max_residual",
        "is_fallback", "fallback_source", "consecutive_fallback_count", "reason",
    ]
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


# -------------------------------------------------------------------- #
# Test 1: All OK frames                                                 #
# -------------------------------------------------------------------- #
class TestAllOKFrames:
    def test_all_ok_metrics(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        _write_poses_csv(csv_path, [
            [0, "0.000000", "OK", "0.10", "0.60", "0.01", 5, "0.020", "0.03", "false", "", 0, "OK"],
            [1, "0.100000", "OK", "0.20", "0.62", "0.02", 5, "0.015", "0.02", "false", "", 0, "OK"],
            [2, "0.200000", "OK", "0.30", "0.63", "0.01", 4, "0.025", "0.04", "false", "", 0, "OK"],
            [3, "0.300000", "OK", "0.40", "0.65", "0.00", 6, "0.010", "0.01", "false", "", 0, "OK"],
        ])
        m = evaluate_poses_from_csv(csv_path)

        assert m.num_frames == 4
        assert m.num_ok == 4
        assert m.num_fallback == 0
        assert m.num_rejected == 0
        assert abs(m.ok_rate - 1.0) < 1e-6
        assert abs(m.fallback_rate - 0.0) < 1e-6
        assert abs(m.rejection_rate - 0.0) < 1e-6
        assert m.mean_residual_rmse is not None
        assert abs(m.mean_residual_rmse - 0.0175) < 1e-4
        assert m.max_residual_rmse is not None
        assert abs(m.max_residual_rmse - 0.025) < 1e-4
        assert m.max_consecutive_fallback == 0
        assert m.drift_warning is False
        assert m.high_fallback_ratio_warning is False
        assert len(m.warnings) == 0


# -------------------------------------------------------------------- #
# Test 2: Mixed OK + Fallback + Rejected                                #
# -------------------------------------------------------------------- #
class TestMixedFrames:
    def test_mixed_metrics(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        _write_poses_csv(csv_path, [
            # Frame 0: OK
            [0, "0.0", "OK", "0.10", "0.60", "0.01", 5, "0.020", "0.03", "false", "", 0, "OK"],
            # Frame 1: Fallback
            [1, "0.1", "FALLBACK_LAST_VALID_POSE", "0.10", "0.60", "0.01", 5, "0.020", "", "true", "last_valid_pose", 1, "Fallback"],
            # Frame 2: Fallback
            [2, "0.2", "FALLBACK_LAST_VALID_POSE", "0.10", "0.60", "0.01", 5, "0.020", "", "true", "last_valid_pose", 2, "Fallback"],
            # Frame 3: OK
            [3, "0.3", "OK", "0.25", "0.61", "0.02", 4, "0.030", "0.05", "false", "", 0, "OK"],
            # Frame 4: Rejected (no pose)
            [4, "0.4", "INSUFFICIENT_DETECTIONS", "", "", "", 0, "", "", "false", "", 0, "Not enough"],
        ])
        m = evaluate_poses_from_csv(csv_path)

        assert m.num_frames == 5
        assert m.num_ok == 2
        assert m.num_fallback == 2
        assert m.num_rejected == 1
        assert abs(m.ok_rate - 0.4) < 1e-6
        assert abs(m.fallback_rate - 0.4) < 1e-6
        assert abs(m.rejection_rate - 0.2) < 1e-6
        assert m.mean_residual_rmse is not None
        assert abs(m.mean_residual_rmse - 0.025) < 1e-4
        assert m.max_consecutive_fallback == 2


# -------------------------------------------------------------------- #
# Test 3: Drift warning triggered (#P2-PREDICT-015)                     #
# -------------------------------------------------------------------- #
class TestDriftWarning:
    def test_drift_warning_triggered(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        rows = []
        # Frame 0: OK
        rows.append([0, "0.0", "OK", "0.1", "0.6", "0.0", 5, "0.02", "", "false", "", 0, "OK"])
        # Frames 1-7: 7 consecutive fallback (> threshold=5)
        for i in range(1, 8):
            rows.append([i, f"{i*0.1:.1f}", "FALLBACK_LAST_VALID_POSE", "0.1", "0.6", "0.0", 5, "0.02", "", "true", "last_valid_pose", i, "Fallback"])
        # Frame 8: OK
        rows.append([8, "0.8", "OK", "0.2", "0.62", "0.01", 4, "0.01", "", "false", "", 0, "OK"])

        _write_poses_csv(csv_path, rows)
        m = evaluate_poses_from_csv(csv_path, drift_threshold=5)

        assert m.max_consecutive_fallback == 7
        assert m.drift_warning is True
        assert any("#P2-PREDICT-015" in w for w in m.warnings)

    def test_drift_warning_not_triggered(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        rows = []
        # Frame 0: OK
        rows.append([0, "0.0", "OK", "0.1", "0.6", "0.0", 5, "0.02", "", "false", "", 0, "OK"])
        # Frames 1-3: 3 consecutive fallback (< threshold=5)
        for i in range(1, 4):
            rows.append([i, f"{i*0.1:.1f}", "FALLBACK_LAST_VALID_POSE", "0.1", "0.6", "0.0", 5, "0.02", "", "true", "last_valid_pose", i, "Fallback"])
        # Frame 4: OK
        rows.append([4, "0.4", "OK", "0.2", "0.62", "0.01", 4, "0.01", "", "false", "", 0, "OK"])

        _write_poses_csv(csv_path, rows)
        m = evaluate_poses_from_csv(csv_path, drift_threshold=5)

        assert m.max_consecutive_fallback == 3
        assert m.drift_warning is False
        assert not any("#P2-PREDICT-015" in w for w in m.warnings)


# -------------------------------------------------------------------- #
# Test 4: High fallback ratio warning (#P2-PREDICT-016)                 #
# -------------------------------------------------------------------- #
class TestHighFallbackRatioWarning:
    def test_high_fallback_ratio(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        rows = []
        # 2 OK + 8 fallback = 80% fallback rate
        for i in range(2):
            rows.append([i, f"{i*0.1:.1f}", "OK", "0.1", "0.6", "0.0", 5, "0.02", "", "false", "", 0, "OK"])
        for i in range(2, 10):
            rows.append([i, f"{i*0.1:.1f}", "FALLBACK_LAST_VALID_POSE", "0.1", "0.6", "0.0", 5, "0.02", "", "true", "last_valid_pose", i-2, "Fallback"])

        _write_poses_csv(csv_path, rows)
        m = evaluate_poses_from_csv(csv_path, fallback_ratio_threshold=0.40)

        assert m.high_fallback_ratio_warning is True
        assert any("#P2-PREDICT-016" in w for w in m.warnings)


# -------------------------------------------------------------------- #
# Test 5: Empty CSV (0 frames)                                         #
# -------------------------------------------------------------------- #
class TestEmptyCSV:
    def test_empty_csv(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        _write_poses_csv(csv_path, [])
        m = evaluate_poses_from_csv(csv_path)

        assert m.num_frames == 0
        assert m.num_ok == 0
        assert m.mean_residual_rmse is None
        assert m.max_residual_rmse is None
        assert m.drift_warning is False
        assert len(m.warnings) == 1
        assert "No frames" in m.warnings[0]


# -------------------------------------------------------------------- #
# Test 6: Zero OK frames but some fallback/rejected                     #
# -------------------------------------------------------------------- #
class TestZeroOK:
    def test_zero_ok_warning(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        rows = [
            [0, "0.0", "INSUFFICIENT_DETECTIONS", "", "", "", 0, "", "", "false", "", 0, "Not enough"],
            [1, "0.1", "ASSOCIATION_FAILED", "", "", "", 0, "", "", "false", "", 0, "No match"],
            [2, "0.2", "DEGENERATE_GEOMETRY", "", "", "", 0, "", "", "false", "", 0, "Degenerate"],
        ]
        _write_poses_csv(csv_path, rows)
        m = evaluate_poses_from_csv(csv_path)

        assert m.num_ok == 0
        assert m.num_rejected == 3
        assert m.mean_residual_rmse is None
        assert any("Zero OK frames" in w for w in m.warnings)


# -------------------------------------------------------------------- #
# Test 7: High residual warning                                         #
# -------------------------------------------------------------------- #
class TestHighResidualWarning:
    def test_high_residual_warning(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        _write_poses_csv(csv_path, [
            [0, "0.0", "OK", "0.1", "0.6", "0.0", 5, "0.15", "", "false", "", 0, "OK"],
            [1, "0.1", "OK", "0.2", "0.6", "0.0", 4, "0.03", "", "false", "", 0, "OK"],
        ])
        m = evaluate_poses_from_csv(csv_path)

        assert m.max_residual_rmse is not None
        assert m.max_residual_rmse > 0.10
        assert any("HIGH_RESIDUAL" in w for w in m.warnings)


# -------------------------------------------------------------------- #
# Test 8: File not found                                                #
# -------------------------------------------------------------------- #
class TestFileNotFound:
    def test_file_not_found(self, tmp_dir):
        with pytest.raises(FileNotFoundError):
            evaluate_poses_from_csv(tmp_dir / "nonexistent.csv")


# -------------------------------------------------------------------- #
# Test 9: evaluate_poses_from_results convenience wrapper               #
# -------------------------------------------------------------------- #
class TestFromResults:
    def test_from_results_dir(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        _write_poses_csv(csv_path, [
            [0, "0.0", "OK", "0.1", "0.6", "0.0", 5, "0.02", "", "false", "", 0, "OK"],
        ])
        m = evaluate_poses_from_results(tmp_dir)
        assert m.num_frames == 1
        assert m.num_ok == 1


# -------------------------------------------------------------------- #
# Test 10: Consecutive streak resets correctly                          #
# -------------------------------------------------------------------- #
class TestStreakReset:
    def test_streak_reset_on_ok(self, tmp_dir):
        csv_path = tmp_dir / "poses.csv"
        rows = [
            [0, "0.0", "FALLBACK_LAST_VALID_POSE", "0.1", "0.6", "0.0", 5, "0.02", "", "true", "last_valid_pose", 1, "FB"],
            [1, "0.1", "FALLBACK_LAST_VALID_POSE", "0.1", "0.6", "0.0", 5, "0.02", "", "true", "last_valid_pose", 2, "FB"],
            [2, "0.2", "OK", "0.15", "0.61", "0.01", 4, "0.01", "", "false", "", 0, "OK"],
            [3, "0.3", "FALLBACK_LAST_VALID_POSE", "0.15", "0.61", "0.01", 4, "0.01", "", "true", "last_valid_pose", 1, "FB"],
        ]
        _write_poses_csv(csv_path, rows)
        m = evaluate_poses_from_csv(csv_path, drift_threshold=5)

        assert m.max_consecutive_fallback == 2  # First streak of 2, then reset, then 1
        assert m.drift_warning is False
