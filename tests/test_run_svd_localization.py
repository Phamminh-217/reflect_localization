"""Integration tests for the offline SVD localization CLI runner."""

import json
from pathlib import Path
import subprocess
import sys
import pytest
import yaml


@pytest.fixture
def setup_synthetic_env(tmp_path: Path) -> tuple:
    """Setup a complete synthetic environment with config, map, and detections."""
    # 1. Config YAML
    cfg_data = {
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
                "max_abs": 0.20,
            },
            "nearest_neighbor_gate": {
                "min_abs": 0.10,
                "relative_ratio": 0.03,
                "max_abs": 0.25,
            },
        },
        "geometry_check": {
            "min_matches": 3,
            "min_spread": 0.10,
            "condition_number": {
                "enabled": True,
                "max_condition_number": 50.0,
                "hard_reject": False,
            },
        },
    }
    cfg_file = tmp_path / "threshold_v1.yaml"
    with cfg_file.open("w", encoding="utf-8") as f:
        yaml.dump(cfg_data, f)

    # 2. Global RF map JSON
    map_data = {
        "map_name": "synthetic_map",
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

    # 3. Detections JSON
    # Frame 0: perfect SVD OK
    # Frame 1: insufficient detections (2) -> recovers via fallback
    # Frame 2: insufficient detections (2) -> recovers via fallback
    # Frame 3: insufficient detections (2) -> fails (consecutive limit is 2)
    detections_data = {
        "detections": [
            {
                "frame_index": 0,
                "stamp": 100.0,
                "frame_id": "livox_frame",
                "num_valid_detections": 3,
                "objects": [
                    {
                        "detection_id": 0,
                        "center_lidar": [0.0, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [-0.1, -0.1, -0.1],
                        "bbox_max": [0.1, 0.1, 0.1],
                    },
                    {
                        "detection_id": 1,
                        "center_lidar": [1.5, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [1.4, -0.1, -0.1],
                        "bbox_max": [1.6, 0.1, 0.1],
                    },
                    {
                        "detection_id": 2,
                        "center_lidar": [0.0, 2.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [-0.1, 1.9, -0.1],
                        "bbox_max": [0.1, 2.1, 0.1],
                    },
                ],
            },
            {
                "frame_index": 1,
                "stamp": 101.0,
                "frame_id": "livox_frame",
                "num_valid_detections": 2,
                "objects": [
                    {
                        "detection_id": 0,
                        "center_lidar": [0.0, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [-0.1, -0.1, -0.1],
                        "bbox_max": [0.1, 0.1, 0.1],
                    },
                    {
                        "detection_id": 1,
                        "center_lidar": [1.5, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [1.4, -0.1, -0.1],
                        "bbox_max": [1.6, 0.1, 0.1],
                    },
                ],
            },
            {
                "frame_index": 2,
                "stamp": 102.0,
                "frame_id": "livox_frame",
                "num_valid_detections": 2,
                "objects": [
                    {
                        "detection_id": 0,
                        "center_lidar": [0.0, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [-0.1, -0.1, -0.1],
                        "bbox_max": [0.1, 0.1, 0.1],
                    },
                    {
                        "detection_id": 1,
                        "center_lidar": [1.5, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [1.4, -0.1, -0.1],
                        "bbox_max": [1.6, 0.1, 0.1],
                    },
                ],
            },
            {
                "frame_index": 3,
                "stamp": 103.0,
                "frame_id": "livox_frame",
                "num_valid_detections": 2,
                "objects": [
                    {
                        "detection_id": 0,
                        "center_lidar": [0.0, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [-0.1, -0.1, -0.1],
                        "bbox_max": [0.1, 0.1, 0.1],
                    },
                    {
                        "detection_id": 1,
                        "center_lidar": [1.5, 0.0, 0.0],
                        "score": 0.9,
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [1.4, -0.1, -0.1],
                        "bbox_max": [1.6, 0.1, 0.1],
                    },
                ],
            },
        ],
    }
    det_file = tmp_path / "detections.json"
    with det_file.open("w", encoding="utf-8") as f:
        json.dump(detections_data, f)

    out_dir = tmp_path / "localization_output"

    return cfg_file, map_file, det_file, out_dir

def test_run_svd_localization_e2e(setup_synthetic_env: tuple) -> None:
    """Verify end-to-end execution of the run_svd_localization.py CLI runner."""
    cfg_file, map_file, det_file, out_dir = setup_synthetic_env

    cmd = [
        sys.executable,
        "scripts/run_svd_localization.py",
        "--config",
        str(cfg_file),
        "--map",
        str(map_file),
        "--detections",
        str(det_file),
        "--output",
        str(out_dir),
    ]

    import os
    env = os.environ.copy()
    env["PYTHONPATH"] = "src"

    res = subprocess.run(cmd, env=env, capture_output=True, text=True, check=True)

    assert res.returncode == 0
    assert "End-to-End SVD Localization completed successfully." in res.stderr or res.stdout

    # Verify that all 8 trace files exist
    assert out_dir.exists()
    assert (out_dir / "poses.csv").exists()
    assert (out_dir / "poses.json").exists()
    assert (out_dir / "rejected_frames.csv").exists()
    assert (out_dir / "localization_summary.csv").exists()
    assert (out_dir / "association_debug.csv").exists()
    assert (out_dir / "svd_debug.csv").exists()
    assert (out_dir / "geometry_debug.csv").exists()
    assert (out_dir / "frame_debug.csv").exists()

    # Read summary csv
    summary_path = out_dir / "localization_summary.csv"
    with summary_path.open("r", encoding="utf-8") as f:
        rows = list(csv_reader := csv_read(f))

    metrics = {r[0]: int(r[1]) for r in rows[1:]}
    assert metrics["num_frames"] == 4
    assert metrics["num_ok"] == 1
    assert metrics["num_fallback"] == 2
    assert metrics["num_rejected_without_fallback"] == 1
    assert metrics["num_insufficient_detections"] == 3


def csv_read(f) -> list:
    """Helper to parse CSV file."""
    import csv
    return csv.reader(f)
