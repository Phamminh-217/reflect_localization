"""Unit tests for the DetectionLoader class."""

import json
from pathlib import Path
import pytest
import numpy as np

from rf_threshold.localization.detection_loader import DetectionLoader, DetectionFrame


def test_detection_loader_success(tmp_path: Path) -> None:
    """Test successful parsing of a valid detections JSON file."""
    detections_data = {
        "detections": [
            {
                "frame_index": 42,
                "stamp": 1716000000.123456,
                "frame_id": "livox_frame",
                "num_valid_detections": 2,
                "objects": [
                    {
                        "detection_id": 0,
                        "center_lidar": [1.2, 0.35, 0.12],
                        "score": 0.82,
                        "num_points": 15,
                        "mean_intensity": 195.0,
                        "max_intensity": 245.0,
                        "bbox_min": [1.16, 0.31, 0.10],
                        "bbox_max": [1.24, 0.39, 0.14],
                    },
                    {
                        "detection_id": 1,
                        "center_lidar": [3.14, -0.5, 0.22],
                        "score": 0.95,
                        "num_points": 24,
                        "mean_intensity": 210.0,
                        "max_intensity": 250.0,
                        "bbox_min": [3.10, -0.55, 0.20],
                        "bbox_max": [3.18, -0.45, 0.24],
                    },
                ],
            }
        ]
    }

    file_path = tmp_path / "detections.json"
    with file_path.open("w", encoding="utf-8") as f:
        json.dump(detections_data, f)

    loader = DetectionLoader(file_path)
    frames = loader.load_frames()

    assert len(frames) == 1
    frame = frames[0]
    assert isinstance(frame, DetectionFrame)
    assert frame.frame_index == 42
    assert frame.stamp == pytest.approx(1716000000.123456)
    assert frame.frame_id == "livox_frame"
    assert len(frame.detections) == 2

    # Verify first detection
    det0 = frame.detections[0]
    assert det0.detection_id == 0
    assert det0.stamp == frame.stamp
    assert det0.frame_id == "livox_frame"
    np.testing.assert_allclose(det0.center_lidar, [1.2, 0.35, 0.12])
    assert det0.score == pytest.approx(0.82)
    assert det0.num_points == 15
    assert det0.mean_intensity == pytest.approx(195.0)
    assert det0.max_intensity == pytest.approx(245.0)
    np.testing.assert_allclose(det0.bbox_min, [1.16, 0.31, 0.10])
    np.testing.assert_allclose(det0.bbox_max, [1.24, 0.39, 0.14])

    # Verify second detection
    det1 = frame.detections[1]
    assert det1.detection_id == 1
    np.testing.assert_allclose(det1.center_lidar, [3.14, -0.5, 0.22])
    assert det1.score == pytest.approx(0.95)


def test_detection_loader_file_not_found() -> None:
    """Test that DetectionLoader raises FileNotFoundError for missing files."""
    loader = DetectionLoader("non_existent_file_path.json")
    with pytest.raises(FileNotFoundError):
        loader.load_frames()


def test_detection_loader_malformed_json(tmp_path: Path) -> None:
    """Test that DetectionLoader raises ValueError for invalid/malformed JSON syntax."""
    file_path = tmp_path / "broken.json"
    with file_path.open("w", encoding="utf-8") as f:
        f.write("{ malformed_json: broken }")

    loader = DetectionLoader(file_path)
    with pytest.raises(ValueError, match="Malformed JSON"):
        loader.load_frames()


def test_detection_loader_invalid_top_level(tmp_path: Path) -> None:
    """Test that DetectionLoader raises ValueError if 'detections' is missing or not a list."""
    file_path = tmp_path / "invalid_top.json"
    with file_path.open("w", encoding="utf-8") as f:
        json.dump({"not_detections": []}, f)

    loader = DetectionLoader(file_path)
    with pytest.raises(ValueError, match="must contain a top-level 'detections' array"):
        loader.load_frames()


def test_detection_loader_missing_frame_keys(tmp_path: Path) -> None:
    """Test that DetectionLoader raises ValueError if a frame lacks required keys."""
    invalid_data = {
        "detections": [
            {
                "frame_index": 0,
                # Missing 'stamp' and 'frame_id'
                "objects": [],
            }
        ]
    }
    file_path = tmp_path / "missing_keys.json"
    with file_path.open("w", encoding="utf-8") as f:
        json.dump(invalid_data, f)

    loader = DetectionLoader(file_path)
    with pytest.raises(ValueError, match="missing required keys"):
        loader.load_frames()


def test_detection_loader_missing_object_keys(tmp_path: Path) -> None:
    """Test that DetectionLoader raises ValueError if an object lacks required fields."""
    invalid_data = {
        "detections": [
            {
                "frame_index": 0,
                "stamp": 1716000000.0,
                "frame_id": "livox_frame",
                "objects": [
                    {
                        "detection_id": 0,
                        # Missing 'center_lidar', 'score', etc.
                    }
                ],
            }
        ]
    }
    file_path = tmp_path / "missing_obj_keys.json"
    with file_path.open("w", encoding="utf-8") as f:
        json.dump(invalid_data, f)

    loader = DetectionLoader(file_path)
    with pytest.raises(ValueError, match="missing keys"):
        loader.load_frames()


def test_detection_loader_invalid_score_range(tmp_path: Path) -> None:
    """Test that DetectionLoader validates the score constraint [0, 1]."""
    invalid_data = {
        "detections": [
            {
                "frame_index": 0,
                "stamp": 1716000000.0,
                "frame_id": "livox_frame",
                "objects": [
                    {
                        "detection_id": 0,
                        "center_lidar": [1.0, 2.0, 3.0],
                        "score": 1.5,  # Invalid (> 1.0)
                        "num_points": 10,
                        "mean_intensity": 100.0,
                        "max_intensity": 200.0,
                        "bbox_min": [0.9, 1.9, 2.9],
                        "bbox_max": [1.1, 2.1, 3.1],
                    }
                ],
            }
        ]
    }
    file_path = tmp_path / "invalid_score.json"
    with file_path.open("w", encoding="utf-8") as f:
        json.dump(invalid_data, f)

    loader = DetectionLoader(file_path)
    with pytest.raises(ValueError, match="score must be in"):
        loader.load_frames()
