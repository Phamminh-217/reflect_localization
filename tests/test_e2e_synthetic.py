"""Phase 2.6.5 — End-to-End Synthetic Integration Test.

This module runs the FULL Phase 2.6 pipeline from start to finish on a controlled
synthetic dataset, verifying that:

  1. The runner (run_svd_localization.py) produces all 8 debug files.
  2. The plot script (plot_localization_debug.py) produces all 5 diagnostic plots.
  3. Metrics in localization_summary.csv match expected values exactly.
  4. Each OK pose in poses.csv can be traced back to matched pairs in association_debug.csv.
  5. Fallback frames are correctly separated from OK frames in the summary.
  6. Frame indices in all debug CSV files are consistent and complete.
  7. No crash when all frames fail consecutively past the fallback limit.
  8. Poses are numerically plausible (finite, within expected range).

Scenario Design (8 frames):
  Frame 0: 3 detections at identity transform → SVD OK
  Frame 1: 3 detections at small translation  → SVD OK
  Frame 2: 2 detections only                  → Fallback (consecutive 1)
  Frame 3: 2 detections only                  → Fallback (consecutive 2, limit is 2)
  Frame 4: 2 detections only                  → Rejected (consecutive 3 > limit)
  Frame 5: 3 detections at identity transform → SVD OK (resets counter)
  Frame 6: 2 detections only                  → Fallback (consecutive 1)
  Frame 7: 3 detections at known transform    → SVD OK
"""

import csv
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import pytest
import yaml


# --------------------------------------------------------------------------- #
# Constants                                                                     #
# --------------------------------------------------------------------------- #

EXPECTED_DEBUG_FILES = [
    "poses.csv",
    "poses.json",
    "rejected_frames.csv",
    "localization_summary.csv",
    "association_debug.csv",
    "svd_debug.csv",
    "geometry_debug.csv",
    "frame_debug.csv",
]

EXPECTED_PLOTS = [
    "01_trajectory.png",
    "02_residuals.png",
    "03_geometry.png",
    "04_status_summary.png",
]

# Map: landmark positions (3 landmarks in a non-collinear configuration)
LANDMARKS = [
    {"id": 0, "position_map": [0.0, 0.0, 0.0]},
    {"id": 1, "position_map": [2.0, 0.0, 0.0]},
    {"id": 2, "position_map": [0.0, 3.0, 0.0]},
]


# --------------------------------------------------------------------------- #
# Synthetic Data Builders                                                       #
# --------------------------------------------------------------------------- #

def _make_detection(det_id: int, cx: float, cy: float, cz: float = 0.12) -> Dict[str, Any]:
    """Return a serializable detection object dict."""
    return {
        "detection_id": det_id,
        "center_lidar": [cx, cy, cz],
        "score": 0.92,
        "num_points": 12,
        "mean_intensity": 180.0,
        "max_intensity": 230.0,
        "bbox_min": [cx - 0.05, cy - 0.05, cz - 0.05],
        "bbox_max": [cx + 0.05, cy + 0.05, cz + 0.05],
    }


def _make_frame(frame_index: int, stamp: float, objects: List[Dict]) -> Dict[str, Any]:
    return {
        "frame_index": frame_index,
        "stamp": stamp,
        "frame_id": "livox_frame",
        "num_valid_detections": len(objects),
        "objects": objects,
    }


def _build_detections_json(tmp_path: Path) -> Path:
    """Build detections.json with 8 synthetic frames."""
    # Identity transform: lidar frame == map frame (robot at origin, yaw=0)
    # So detected lidar positions equal map landmark positions
    lm0 = [LANDMARKS[0]["position_map"][0], LANDMARKS[0]["position_map"][1]]  # [0, 0]
    lm1 = [LANDMARKS[1]["position_map"][0], LANDMARKS[1]["position_map"][1]]  # [2, 0]
    lm2 = [LANDMARKS[2]["position_map"][0], LANDMARKS[2]["position_map"][1]]  # [0, 3]

    # Frame 1: robot translated by [0.5, 0.2], yaw=0
    # lidar_positions = map_positions - translation = lm - [0.5, 0.2]
    t_x, t_y = 0.5, 0.2
    lm0_t1 = [lm0[0] - t_x, lm0[1] - t_y]
    lm1_t1 = [lm1[0] - t_x, lm1[1] - t_y]
    lm2_t1 = [lm2[0] - t_x, lm2[1] - t_y]

    # Frame 7: robot at [1.0, 1.0], yaw=0
    t_x7, t_y7 = 1.0, 1.0
    lm0_t7 = [lm0[0] - t_x7, lm0[1] - t_y7]
    lm1_t7 = [lm1[0] - t_x7, lm1[1] - t_y7]
    lm2_t7 = [lm2[0] - t_x7, lm2[1] - t_y7]

    frames = [
        # Frame 0: OK (identity)
        _make_frame(0, 100.0, [
            _make_detection(0, lm0[0], lm0[1]),
            _make_detection(1, lm1[0], lm1[1]),
            _make_detection(2, lm2[0], lm2[1]),
        ]),
        # Frame 1: OK (translation [0.5, 0.2])
        _make_frame(1, 101.0, [
            _make_detection(0, lm0_t1[0], lm0_t1[1]),
            _make_detection(1, lm1_t1[0], lm1_t1[1]),
            _make_detection(2, lm2_t1[0], lm2_t1[1]),
        ]),
        # Frame 2: 2 detections → Fallback (consecutive=1)
        _make_frame(2, 102.0, [
            _make_detection(0, lm0[0], lm0[1]),
            _make_detection(1, lm1[0], lm1[1]),
        ]),
        # Frame 3: 2 detections → Fallback (consecutive=2, at limit)
        _make_frame(3, 103.0, [
            _make_detection(0, lm0[0], lm0[1]),
            _make_detection(1, lm1[0], lm1[1]),
        ]),
        # Frame 4: 2 detections → Rejected (consecutive=3 > limit=2)
        _make_frame(4, 104.0, [
            _make_detection(0, lm0[0], lm0[1]),
            _make_detection(1, lm1[0], lm1[1]),
        ]),
        # Frame 5: 3 detections OK (identity, resets counter)
        _make_frame(5, 105.0, [
            _make_detection(0, lm0[0], lm0[1]),
            _make_detection(1, lm1[0], lm1[1]),
            _make_detection(2, lm2[0], lm2[1]),
        ]),
        # Frame 6: 2 detections → Fallback (consecutive=1 again)
        _make_frame(6, 106.0, [
            _make_detection(0, lm0[0], lm0[1]),
            _make_detection(1, lm1[0], lm1[1]),
        ]),
        # Frame 7: 3 detections OK (translation [1.0, 1.0])
        _make_frame(7, 107.0, [
            _make_detection(0, lm0_t7[0], lm0_t7[1]),
            _make_detection(1, lm1_t7[0], lm1_t7[1]),
            _make_detection(2, lm2_t7[0], lm2_t7[1]),
        ]),
    ]

    data = {"detections": frames}
    path = tmp_path / "detections.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return path


def _build_rf_map(tmp_path: Path) -> Path:
    data = {
        "map_name": "e2e_synthetic_map",
        "frame_id": "map_frame",
        "unit": "meter",
        "landmarks": LANDMARKS,
    }
    path = tmp_path / "rf_map_v1.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return path


def _build_config(tmp_path: Path) -> Path:
    cfg = {
        "fallback": {
            "enabled": True,
            "mode": "last_valid_pose",
            "max_consecutive_fallback_frames": 2,
        },
        "data_association": {
            "min_detections": 3,
            "min_matches": 3,
            "max_candidate_rmse": 0.08,
            "max_candidate_residual": 0.18,
            "triplet_distance_tolerance": {
                "min_abs": 0.08,
                "relative_ratio": 0.03,
                "max_abs": 0.25,
            },
            "nearest_neighbor_gate": {
                "min_abs": 0.10,
                "relative_ratio": 0.03,
                "max_abs": 0.30,
            },
        },
        "geometry_check": {
            "min_matches": 3,
            "min_spread": 0.10,
            "condition_number": {
                "enabled": True,
                "max_condition_number": 100.0,
                "hard_reject": False,
            },
        },
    }
    path = tmp_path / "threshold_v1.yaml"
    with path.open("w", encoding="utf-8") as f:
        yaml.dump(cfg, f)
    return path


# --------------------------------------------------------------------------- #
# Helpers                                                                       #
# --------------------------------------------------------------------------- #

def _load_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _safe_float(val: str) -> Optional[float]:
    try:
        return float(val)
    except (TypeError, ValueError):
        return None


def _run_runner(cfg_file: Path, map_file: Path, det_file: Path, out_dir: Path) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["PYTHONPATH"] = "src"
    cmd = [
        sys.executable, "scripts/run_svd_localization.py",
        "--config", str(cfg_file),
        "--map", str(map_file),
        "--detections", str(det_file),
        "--output", str(out_dir),
    ]
    return subprocess.run(cmd, env=env, capture_output=True, text=True)


def _run_plot(out_dir: Path, map_file: Path, save_dir: Path) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["PYTHONPATH"] = "src"
    env["MPLBACKEND"] = "Agg"
    cmd = [
        sys.executable, "scripts/plot_localization_debug.py",
        "--output", str(out_dir),
        "--map", str(map_file),
        "--save", str(save_dir),
    ]
    return subprocess.run(cmd, env=env, capture_output=True, text=True)


# --------------------------------------------------------------------------- #
# Shared Fixture                                                                #
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="module")
def e2e_env(tmp_path_factory):
    """Build the full synthetic environment and run the pipeline once.

    Returns a dict with all paths and subprocess results.
    """
    tmp_path = tmp_path_factory.mktemp("e2e")
    det_file = _build_detections_json(tmp_path)
    map_file = _build_rf_map(tmp_path)
    cfg_file = _build_config(tmp_path)
    out_dir = tmp_path / "localization"
    plot_dir = tmp_path / "plots"

    runner_res = _run_runner(cfg_file, map_file, det_file, out_dir)
    plot_res = _run_plot(out_dir, map_file, plot_dir)

    return {
        "tmp_path": tmp_path,
        "det_file": det_file,
        "map_file": map_file,
        "cfg_file": cfg_file,
        "out_dir": out_dir,
        "plot_dir": plot_dir,
        "runner_res": runner_res,
        "plot_res": plot_res,
    }


# --------------------------------------------------------------------------- #
# Test Cases                                                                    #
# --------------------------------------------------------------------------- #

class TestE2ERunnerExit:
    """Verify runner and plot script exit codes."""

    def test_runner_exits_zero(self, e2e_env):
        """Runner must exit with code 0 on success."""
        res = e2e_env["runner_res"]
        assert res.returncode == 0, (
            f"Runner failed (returncode={res.returncode}).\n"
            f"STDERR:\n{res.stderr}\n"
            f"STDOUT:\n{res.stdout}"
        )

    def test_runner_logs_completion(self, e2e_env):
        """Runner stdout must contain the completion message."""
        combined = e2e_env["runner_res"].stdout + e2e_env["runner_res"].stderr
        assert "End-to-End SVD Localization completed successfully." in combined

    def test_plot_exits_zero(self, e2e_env):
        """Plot script must exit with code 0."""
        res = e2e_env["plot_res"]
        assert res.returncode == 0, (
            f"Plot script failed (returncode={res.returncode}).\n"
            f"STDERR:\n{res.stderr}\n"
            f"STDOUT:\n{res.stdout}"
        )


class TestE2EOutputFiles:
    """Verify all 8 debug CSV/JSON files exist and are non-empty."""

    def test_all_8_debug_files_exist(self, e2e_env):
        out_dir = e2e_env["out_dir"]
        for fname in EXPECTED_DEBUG_FILES:
            fpath = out_dir / fname
            assert fpath.exists(), f"Missing expected debug file: {fname}"
            assert fpath.stat().st_size > 0, f"Debug file is empty: {fname}"

    def test_all_plots_exist(self, e2e_env):
        plot_dir = e2e_env["plot_dir"]
        for fname in EXPECTED_PLOTS:
            fpath = plot_dir / fname
            assert fpath.exists(), f"Missing expected plot: {fname}"
            assert fpath.stat().st_size > 0, f"Plot file is empty: {fname}"


class TestE2ESummaryMetrics:
    """Verify localization_summary.csv contains correct metrics for the 8-frame scenario."""

    def _get_summary(self, e2e_env) -> Dict[str, int]:
        rows = _load_csv(e2e_env["out_dir"] / "localization_summary.csv")
        return {row["metric"]: int(row["value"]) for row in rows}

    def test_total_frame_count(self, e2e_env):
        summary = self._get_summary(e2e_env)
        assert summary["num_frames"] == 8, f"Expected 8 frames, got {summary['num_frames']}"

    def test_ok_frame_count(self, e2e_env):
        """Frames 0, 1, 5, 7 should be SVD OK."""
        summary = self._get_summary(e2e_env)
        assert summary["num_ok"] == 4, f"Expected 4 OK frames, got {summary['num_ok']}"

    def test_fallback_frame_count(self, e2e_env):
        """Frames 2, 3, 6 should be fallback (limit=2: frame 4 exceeds limit)."""
        summary = self._get_summary(e2e_env)
        assert summary["num_fallback"] == 3, (
            f"Expected 3 fallback frames, got {summary['num_fallback']}"
        )

    def test_rejected_without_fallback_count(self, e2e_env):
        """Frame 4 should be rejected (consecutive=3 > limit=2)."""
        summary = self._get_summary(e2e_env)
        assert summary["num_rejected_without_fallback"] == 1, (
            f"Expected 1 rejected-without-fallback frame, got {summary['num_rejected_without_fallback']}"
        )

    def test_fallback_not_counted_as_ok(self, e2e_env):
        """num_ok + num_fallback + num_rejected must equal num_frames."""
        summary = self._get_summary(e2e_env)
        total = summary["num_ok"] + summary["num_fallback"] + summary["num_rejected_without_fallback"]
        assert total == summary["num_frames"], (
            f"OK + Fallback + Rejected = {total} ≠ num_frames = {summary['num_frames']}. "
            "Fallback frames must NOT be counted in num_ok."
        )

    def test_insufficient_detections_count(self, e2e_env):
        """Frames 2, 3, 4, 6 each have 2 detections < min 3."""
        summary = self._get_summary(e2e_env)
        assert summary["num_insufficient_detections"] == 4, (
            f"Expected 4 INSUFFICIENT_DETECTIONS, got {summary['num_insufficient_detections']}"
        )


class TestE2EPosesCSV:
    """Verify structure and content of poses.csv."""

    def _get_poses(self, e2e_env) -> List[Dict[str, str]]:
        return _load_csv(e2e_env["out_dir"] / "poses.csv")

    def test_poses_csv_has_8_rows(self, e2e_env):
        """poses.csv must have 8 data rows (one per frame)."""
        rows = self._get_poses(e2e_env)
        assert len(rows) == 8, f"Expected 8 rows in poses.csv, got {len(rows)}"

    def test_ok_frames_have_pose_values(self, e2e_env):
        """Frames 0, 1, 5, 7 must have numeric x/y/yaw values."""
        rows = self._get_poses(e2e_env)
        ok_frames = {int(r["frame_index"]): r for r in rows if r["status"] == "OK"}
        for fi in [0, 1, 5, 7]:
            assert fi in ok_frames, f"Frame {fi} not found in OK frames"
            row = ok_frames[fi]
            assert _safe_float(row["x"]) is not None, f"Frame {fi} x is missing"
            assert _safe_float(row["y"]) is not None, f"Frame {fi} y is missing"
            assert _safe_float(row["yaw"]) is not None, f"Frame {fi} yaw is missing"

    def test_fallback_frames_have_pose_values(self, e2e_env):
        """Frames 2, 3, 6 must have pose values (borrowed from fallback)."""
        rows = self._get_poses(e2e_env)
        fb_rows = {int(r["frame_index"]): r for r in rows
                   if r.get("is_fallback", "false") == "true"}
        for fi in [2, 3, 6]:
            assert fi in fb_rows, f"Frame {fi} not found in fallback rows"
            row = fb_rows[fi]
            assert _safe_float(row["x"]) is not None, f"Fallback frame {fi} x is missing"

    def test_rejected_frame_4_has_no_pose(self, e2e_env):
        """Frame 4 must have empty x/y/yaw (pure rejection, no fallback available)."""
        rows = self._get_poses(e2e_env)
        row4 = next((r for r in rows if int(r["frame_index"]) == 4), None)
        assert row4 is not None, "Frame 4 not found in poses.csv"
        assert _safe_float(row4["x"]) is None, "Frame 4 x should be empty (rejected)"
        assert _safe_float(row4["y"]) is None, "Frame 4 y should be empty (rejected)"

    def test_poses_residuals_finite_and_non_negative(self, e2e_env):
        """All OK/Fallback frames with residual must have finite non-negative residuals."""
        rows = self._get_poses(e2e_env)
        for row in rows:
            r = _safe_float(row.get("residual_rmse", ""))
            if r is not None:
                assert math.isfinite(r), f"Frame {row['frame_index']}: residual_rmse is not finite"
                assert r >= 0.0, f"Frame {row['frame_index']}: residual_rmse is negative"

    def test_ok_frame0_pose_near_identity(self, e2e_env):
        """Frame 0 with identity transform: x, y should be near 0."""
        rows = self._get_poses(e2e_env)
        row0 = next((r for r in rows if int(r["frame_index"]) == 0), None)
        assert row0 is not None, "Frame 0 not found"
        x = _safe_float(row0["x"])
        y = _safe_float(row0["y"])
        assert x is not None and abs(x) < 0.05, f"Frame 0 x={x} not near 0"
        assert y is not None and abs(y) < 0.05, f"Frame 0 y={y} not near 0"

    def test_ok_frame1_pose_near_translation(self, e2e_env):
        """Frame 1 with translation [0.5, 0.2]: x≈0.5, y≈0.2."""
        rows = self._get_poses(e2e_env)
        row1 = next((r for r in rows if int(r["frame_index"]) == 1), None)
        assert row1 is not None, "Frame 1 not found"
        x = _safe_float(row1["x"])
        y = _safe_float(row1["y"])
        assert x is not None and abs(x - 0.5) < 0.05, f"Frame 1 x={x} not near 0.5"
        assert y is not None and abs(y - 0.2) < 0.05, f"Frame 1 y={y} not near 0.2"

    def test_ok_frame7_pose_near_translation(self, e2e_env):
        """Frame 7 with translation [1.0, 1.0]: x≈1.0, y≈1.0."""
        rows = self._get_poses(e2e_env)
        row7 = next((r for r in rows if int(r["frame_index"]) == 7), None)
        assert row7 is not None, "Frame 7 not found"
        x = _safe_float(row7["x"])
        y = _safe_float(row7["y"])
        assert x is not None and abs(x - 1.0) < 0.05, f"Frame 7 x={x} not near 1.0"
        assert y is not None and abs(y - 1.0) < 0.05, f"Frame 7 y={y} not near 1.0"


class TestE2ETraceability:
    """Verify cross-file traceability: all frame_indices are consistent."""

    def _all_frame_indices(self, rows: List[Dict[str, str]]) -> List[int]:
        return [int(r["frame_index"]) for r in rows]

    def test_poses_frame_debug_consistent(self, e2e_env):
        """frame_debug.csv must have the same frame indices as poses.csv."""
        poses = _load_csv(e2e_env["out_dir"] / "poses.csv")
        frame_debug = _load_csv(e2e_env["out_dir"] / "frame_debug.csv")
        assert sorted(self._all_frame_indices(poses)) == sorted(
            self._all_frame_indices(frame_debug)
        ), "Frame indices differ between poses.csv and frame_debug.csv"

    def test_ok_frames_have_association_rows(self, e2e_env):
        """Each OK frame must have at least one row in association_debug.csv."""
        poses = _load_csv(e2e_env["out_dir"] / "poses.csv")
        assoc = _load_csv(e2e_env["out_dir"] / "association_debug.csv")
        assoc_frames = {int(r["frame_index"]) for r in assoc}
        ok_frames = [int(r["frame_index"]) for r in poses if r["status"] == "OK"]
        for fi in ok_frames:
            assert fi in assoc_frames, (
                f"OK frame {fi} has no rows in association_debug.csv"
            )

    def test_rejected_frames_in_rejected_csv(self, e2e_env):
        """Frame 4 (pure rejected) must appear in rejected_frames.csv."""
        rejected = _load_csv(e2e_env["out_dir"] / "rejected_frames.csv")
        rej_indices = {int(r["frame_index"]) for r in rejected}
        assert 4 in rej_indices, "Frame 4 (pure rejected) not in rejected_frames.csv"

    def test_fallback_frames_in_rejected_csv(self, e2e_env):
        """Fallback frames (2, 3, 6) must also appear in rejected_frames.csv (with fallback_used=true)."""
        rejected = _load_csv(e2e_env["out_dir"] / "rejected_frames.csv")
        for row in rejected:
            fi = int(row["frame_index"])
            if fi in [2, 3, 6]:
                assert row.get("fallback_used", "") == "true", (
                    f"Frame {fi} is a fallback frame but fallback_used!=true in rejected_frames.csv"
                )

    def test_svd_debug_ok_frames_have_rotation_matrix(self, e2e_env):
        """OK frames in svd_debug.csv must have non-empty R00 values."""
        svd = _load_csv(e2e_env["out_dir"] / "svd_debug.csv")
        poses = _load_csv(e2e_env["out_dir"] / "poses.csv")
        ok_frames = {int(r["frame_index"]) for r in poses if r["status"] == "OK"}
        for row in svd:
            fi = int(row["frame_index"])
            if fi in ok_frames:
                assert row.get("R00", "") != "", (
                    f"OK frame {fi} in svd_debug.csv has empty R00"
                )
                r00 = _safe_float(row["R00"])
                assert r00 is not None and math.isfinite(r00), (
                    f"OK frame {fi} R00 is not finite: {row['R00']}"
                )

    def test_poses_json_matches_poses_csv_count(self, e2e_env):
        """poses.json must contain the same number of pose entries as poses.csv rows."""
        poses_csv = _load_csv(e2e_env["out_dir"] / "poses.csv")
        poses_json = _load_json(e2e_env["out_dir"] / "poses.json")
        assert len(poses_json["poses"]) == len(poses_csv), (
            f"poses.json has {len(poses_json['poses'])} entries "
            f"but poses.csv has {len(poses_csv)} rows"
        )


class TestE2EEdgeCases:
    """Verify edge case handling in the full pipeline."""

    def test_runner_fails_gracefully_on_missing_detections(self, tmp_path):
        """Runner must exit non-zero if detections file is missing."""
        cfg = tmp_path / "cfg.yaml"
        with cfg.open("w") as f:
            yaml.dump({"fallback": {"enabled": True}}, f)
        map_f = tmp_path / "map.json"
        with map_f.open("w") as f:
            json.dump({"landmarks": []}, f)

        env = os.environ.copy()
        env["PYTHONPATH"] = "src"
        res = subprocess.run(
            [
                sys.executable, "scripts/run_svd_localization.py",
                "--config", str(cfg),
                "--map", str(map_f),
                "--detections", str(tmp_path / "nonexistent_detections.json"),
                "--output", str(tmp_path / "out"),
            ],
            env=env, capture_output=True, text=True,
        )
        assert res.returncode != 0, "Expected non-zero exit when detections file is missing"

    def test_runner_fails_gracefully_on_empty_config(self, tmp_path):
        """Runner must handle empty YAML config and exit with error."""
        cfg = tmp_path / "empty.yaml"
        with cfg.open("w") as f:
            f.write("null\n")
        det = tmp_path / "det.json"
        with det.open("w") as f:
            json.dump({"detections": []}, f)
        map_f = tmp_path / "map.json"
        with map_f.open("w") as f:
            json.dump({"landmarks": LANDMARKS}, f)

        env = os.environ.copy()
        env["PYTHONPATH"] = "src"
        res = subprocess.run(
            [
                sys.executable, "scripts/run_svd_localization.py",
                "--config", str(cfg),
                "--map", str(map_f),
                "--detections", str(det),
                "--output", str(tmp_path / "out"),
            ],
            env=env, capture_output=True, text=True,
        )
        assert res.returncode != 0, "Expected non-zero exit when config is null/empty"
