"""Module for parsing ROS PointCloud2 messages into internal LidarFrame."""

import logging
from typing import Any

import numpy as np
import sensor_msgs.point_cloud2 as pc2

from rf_threshold.core.frame import LidarFrame

logger = logging.getLogger("pointcloud_parser")


def parse_pointcloud_message(msg: Any) -> LidarFrame:
    """Parse a ROS PointCloud2 message into a LidarFrame object.

    Args:
        msg: The ROS message, expected to be of type sensor_msgs/PointCloud2.

    Returns:
        A LidarFrame object containing point coordinates and intensity values.

    Raises:
        TypeError: If the input message is not of type sensor_msgs/PointCloud2.
        ValueError: If required fields (x, y, z, intensity) are missing or invalid.
    """
    # 1. Verify message type
    # Check if the message is a PointCloud2 message by its _type attribute
    if not hasattr(msg, "_type") or msg._type != "sensor_msgs/PointCloud2":
        raise TypeError(
            f"Unsupported message type: {type(msg)}. "
            "Expected sensor_msgs/PointCloud2."
        )

    # 2. Check for required fields
    fields = [field.name for field in msg.fields]
    for required in ("x", "y", "z", "intensity"):
        if required not in fields:
            raise ValueError(
                f"Required field '{required}' not found in point cloud message fields: {fields}"
            )

    # 3. Extract stamp and frame_id from header
    if not hasattr(msg, "header") or msg.header is None:
        raise ValueError("PointCloud2 message is missing header.")

    stamp_sec = msg.header.stamp.to_sec()
    frame_id = msg.header.frame_id

    # 4. Parse point cloud data using ros point_cloud2 generator
    try:
        points_gen = pc2.read_points(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=False,
        )
        points_list = list(points_gen)
    except Exception as exc:
        logger.error("Failed to read points from PointCloud2: %s", exc)
        raise ValueError(f"Error parsing point cloud bytes: {exc}") from exc

    # 5. Convert to numpy arrays
    if len(points_list) == 0:
        points_xyz = np.empty((0, 3), dtype=np.float64)
        intensity = np.empty((0,), dtype=np.float64)
    else:
        points_arr = np.array(points_list, dtype=np.float64)
        points_xyz = points_arr[:, :3]
        intensity = points_arr[:, 3]

    # Create LidarFrame object (validation is performed in __post_init__)
    return LidarFrame(
        stamp=stamp_sec,
        frame_id=frame_id,
        points_xyz=points_xyz,
        intensity=intensity,
    )
