"""Module for loading and parsing the reflective feature (RF) map."""

import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Union
import numpy as np

from rf_threshold.localization.pose import RFMapLandmark

logger = logging.getLogger("map_loader")


def load_rf_map(map_path: Union[str, Path]) -> List[RFMapLandmark]:
    """Load a reflective feature map from a JSON file.

    Args:
        map_path: Path to the JSON map file.

    Returns:
        A list of validated RFMapLandmark objects.

    Raises:
        FileNotFoundError: If the map file does not exist.
        ValueError: If JSON is invalid, missing required fields, or landmark positions are invalid.
        KeyError: If required JSON structure keys are missing.
    """
    path = Path(map_path)
    if not path.is_file():
        logger.error("Map file not found: %s", path)
        raise FileNotFoundError(f"Map file not found: {path}")

    logger.info("Loading RF map from: %s", path)
    try:
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)
    except json.JSONDecodeError as exc:
        logger.error("Failed to parse JSON in map file: %s", exc)
        raise ValueError(f"Invalid JSON format in map file: {exc}") from exc

    if "landmarks" not in data:
        logger.error("Map JSON lacks required key 'landmarks'")
        raise KeyError("Missing required key 'landmarks' in map file")

    landmarks_data = data["landmarks"]
    if not isinstance(landmarks_data, list):
        logger.error("'landmarks' key must map to a JSON list")
        raise TypeError("'landmarks' must be a list in map file")

    landmarks: List[RFMapLandmark] = []
    for idx, item in enumerate(landmarks_data):
        if not isinstance(item, dict):
            logger.error("Landmark item at index %d is not a JSON object", idx)
            raise TypeError(f"Landmark at index {idx} must be a dictionary")

        if "id" not in item:
            logger.error("Landmark item at index %d is missing 'id'", idx)
            raise KeyError(f"Landmark at index {idx} is missing required key 'id'")

        if "position_map" not in item:
            logger.error("Landmark item at index %d is missing 'position_map'", idx)
            raise KeyError(f"Landmark at index {idx} is missing required key 'position_map'")

        landmark_id = item["id"]
        position_list = item["position_map"]
        frame_id = item.get("frame_id", "map_frame")

        if not isinstance(position_list, list):
            logger.error("Landmark ID %d 'position_map' is not a list", landmark_id)
            raise TypeError(f"Landmark ID {landmark_id} position_map must be a list")

        # Convert to numpy array
        try:
            position_arr = np.array(position_list, dtype=np.float64)
        except (ValueError, TypeError) as exc:
            logger.error("Landmark ID %d position conversion failed", landmark_id)
            raise ValueError(f"Landmark ID {landmark_id} position cannot be converted to floats: {exc}") from exc

        # Create RFMapLandmark (validation of shape, NaN, and positive ID is handled in RFMapLandmark.__post_init__)
        try:
            landmark = RFMapLandmark(
                landmark_id=int(landmark_id),
                position_map=position_arr,
                frame_id=str(frame_id),
            )
        except Exception as exc:
            logger.error("Landmark validation failed for ID %d: %s", landmark_id, exc)
            raise ValueError(f"Failed to create RFMapLandmark for ID {landmark_id}: {exc}") from exc

        landmarks.append(landmark)

    logger.info("Successfully loaded %d landmarks from map.", len(landmarks))
    return landmarks
