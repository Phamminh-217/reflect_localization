"""Module for loading serialized RF detection results from JSON formats."""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Union

import numpy as np

from rf_threshold.core.frame import RFDetection


@dataclass(frozen=True)
class DetectionFrame:
    """Represent one LiDAR frame containing all parsed valid detections.

    Args:
        frame_index: Index of the current frame.
        stamp: Timestamp of the current frame in seconds.
        frame_id: Name of the LiDAR coordinate frame.
        detections: List of observed RF detections in the LiDAR frame.
    """

    frame_index: int
    stamp: float
    frame_id: str
    detections: List[RFDetection]


class DetectionLoader:
    """Loader to parse detections.json and convert them to memory-resident objects."""

    def __init__(self, file_path: Union[str, Path]) -> None:
        """Initialize the detection loader.

        Args:
            file_path: Path to the detections JSON file.
        """
        self.file_path = Path(file_path)

    def load_frames(self) -> List[DetectionFrame]:
        """Load all frames and their valid detections from the JSON file.

        Returns:
            A list of DetectionFrame objects.

        Raises:
            FileNotFoundError: If the detections file does not exist.
            ValueError: If the JSON is malformed or missing key fields.
        """
        if not self.file_path.exists():
            raise FileNotFoundError(f"Detections file not found: {self.file_path}")

        try:
            with self.file_path.open("r", encoding="utf-8") as file:
                data = json.load(file)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Malformed JSON in detections file: {exc}") from exc
        except Exception as exc:
            raise ValueError(f"Failed to read detections file: {exc}") from exc

        if not isinstance(data, dict) or "detections" not in data:
            raise ValueError("Detections file must contain a top-level 'detections' array.")

        frames_list = data["detections"]
        if not isinstance(frames_list, list):
            raise ValueError("Top-level 'detections' key must map to a list.")

        parsed_frames: List[DetectionFrame] = []

        for f_idx, frame in enumerate(frames_list):
            if not isinstance(frame, dict):
                raise ValueError(f"Frame at list index {f_idx} is not a valid dictionary.")

            required_frame_keys = {"frame_index", "stamp", "frame_id", "objects"}
            missing_frame_keys = required_frame_keys - frame.keys()
            if missing_frame_keys:
                raise ValueError(
                    f"Frame at list index {f_idx} is missing required keys: {missing_frame_keys}"
                )

            frame_index = frame["frame_index"]
            stamp = frame["stamp"]
            frame_id = frame["frame_id"]
            objects_list = frame["objects"]

            if not isinstance(objects_list, list):
                raise ValueError(f"'objects' field in frame {frame_index} must be a list.")

            detections: List[RFDetection] = []

            for o_idx, obj in enumerate(objects_list):
                if not isinstance(obj, dict):
                    raise ValueError(
                        f"Object at index {o_idx} in frame {frame_index} is not a dictionary."
                    )

                required_obj_keys = {
                    "detection_id",
                    "center_lidar",
                    "score",
                    "num_points",
                    "mean_intensity",
                    "max_intensity",
                    "bbox_min",
                    "bbox_max",
                }
                missing_obj_keys = required_obj_keys - obj.keys()
                if missing_obj_keys:
                    raise ValueError(
                        f"Object in frame {frame_index} at index {o_idx} is missing keys: {missing_obj_keys}"
                    )

                try:
                    center_lidar = np.array(obj["center_lidar"], dtype=np.float64)
                    bbox_min = np.array(obj["bbox_min"], dtype=np.float64)
                    bbox_max = np.array(obj["bbox_max"], dtype=np.float64)

                    det = RFDetection(
                        detection_id=int(obj["detection_id"]),
                        stamp=float(stamp),
                        frame_id=str(frame_id),
                        center_lidar=center_lidar,
                        score=float(obj["score"]),
                        num_points=int(obj["num_points"]),
                        mean_intensity=float(obj["mean_intensity"]),
                        max_intensity=float(obj["max_intensity"]),
                        bbox_min=bbox_min,
                        bbox_max=bbox_max,
                        cluster_id=obj.get("cluster_id"),
                    )
                    detections.append(det)
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"Invalid type or value in object {o_idx} of frame {frame_index}: {exc}"
                    ) from exc

            parsed_frames.append(
                DetectionFrame(
                    frame_index=int(frame_index),
                    stamp=float(stamp),
                    frame_id=str(frame_id),
                    detections=detections,
                )
            )

        return parsed_frames
