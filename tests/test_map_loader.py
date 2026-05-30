"""Unit tests for RF map loader."""

import json
import pytest
import numpy as np

from rf_threshold.localization.map_loader import load_rf_map


def test_load_valid_map_file():
    """Test loading the actual standard map file in the repository."""
    map_path = "data/maps/your_map_simple.json"
    landmarks = load_rf_map(map_path)
    assert len(landmarks) == 18
    
    # Verify the first landmark
    landmark_0 = landmarks[0]
    assert landmark_0.landmark_id == 0
    assert np.array_equal(landmark_0.position_map, np.array([0.0, 0.0, 1.3]))
    assert landmark_0.frame_id == "map_frame"

    # Verify a random landmark mid-list
    landmark_10 = landmarks[10]
    assert landmark_10.landmark_id == 10
    assert np.array_equal(landmark_10.position_map, np.array([1.41, 3.005, 1.3]))


def test_load_non_existent_file():
    """Test that a non-existent map path raises FileNotFoundError."""
    with pytest.raises(FileNotFoundError):
        load_rf_map("data/maps/does_not_exist.json")


def test_load_invalid_json(tmp_path):
    """Test that malformed JSON raises ValueError."""
    bad_json_file = tmp_path / "bad_map.json"
    bad_json_file.write_text("{ invalid json ... }")
    with pytest.raises(ValueError, match="Invalid JSON format"):
        load_rf_map(bad_json_file)


def test_load_missing_landmarks_key(tmp_path):
    """Test that JSON missing 'landmarks' raises KeyError."""
    no_landmarks_file = tmp_path / "no_landmarks.json"
    no_landmarks_file.write_text(json.dumps({"format_version": "1.0"}))
    with pytest.raises(KeyError, match="Missing required key 'landmarks'"):
        load_rf_map(no_landmarks_file)


def test_load_landmarks_not_a_list(tmp_path):
    """Test that 'landmarks' not mapping to a list raises TypeError."""
    bad_type_file = tmp_path / "bad_type.json"
    bad_type_file.write_text(json.dumps({"landmarks": "not a list"}))
    with pytest.raises(TypeError, match="must be a list"):
        load_rf_map(bad_type_file)


def test_load_landmark_missing_id(tmp_path):
    """Test that a landmark item missing 'id' raises KeyError."""
    missing_id_file = tmp_path / "missing_id.json"
    missing_id_file.write_text(json.dumps({
        "landmarks": [{"position_map": [1.0, 2.0, 3.0]}]
    }))
    with pytest.raises(KeyError, match="missing required key 'id'"):
        load_rf_map(missing_id_file)


def test_load_landmark_missing_position(tmp_path):
    """Test that a landmark item missing 'position_map' raises KeyError."""
    missing_pos_file = tmp_path / "missing_pos.json"
    missing_pos_file.write_text(json.dumps({
        "landmarks": [{"id": 0}]
    }))
    with pytest.raises(KeyError, match="missing required key 'position_map'"):
        load_rf_map(missing_pos_file)


def test_load_landmark_invalid_position_shape(tmp_path):
    """Test that an incorrect position array shape raises ValueError."""
    bad_shape_file = tmp_path / "bad_shape.json"
    bad_shape_file.write_text(json.dumps({
        "landmarks": [{"id": 0, "position_map": [1.0, 2.0]}]
    }))
    with pytest.raises(ValueError, match="position_map must have shape"):
        load_rf_map(bad_shape_file)


def test_load_landmark_contains_nan(tmp_path):
    """Test that position coordinates containing NaN/Inf raise ValueError."""
    nan_file = tmp_path / "nan_pos.json"
    nan_file.write_text(json.dumps({
        "landmarks": [{"id": 0, "position_map": [1.0, float("nan"), 3.0]}]
    }))
    with pytest.raises(ValueError, match="contains NaN or Inf"):
        load_rf_map(nan_file)
