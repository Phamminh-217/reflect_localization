"""Module for reading LiDAR frames sequentially from a ROS bag."""

import logging
from pathlib import Path
from typing import Any, Dict, Iterator, Union

import rosbag

from rf_threshold.core.frame import LidarFrame
from rf_threshold.io.pointcloud_parser import parse_pointcloud_message

logger = logging.getLogger("bag_reader")


def read_lidar_frames(
    bag_path: Union[str, Path],
    topic: str,
    cfg: Dict[str, Any],
) -> Iterator[LidarFrame]:
    """Read a ROS bag file and yield LidarFrame objects from the specified topic.

    Args:
        bag_path: Path to the ROS bag file.
        topic: The topic name containing LiDAR point cloud messages.
        cfg: Configuration dictionary (unused for now, but reserved for expansion).

    Yields:
        LidarFrame objects parsed from the bag file.

    Raises:
        FileNotFoundError: If the bag file does not exist.
        ValueError: If the bag file is invalid, or if the specified topic
            has no messages in the bag.
    """
    path = Path(bag_path)

    if not path.exists():
        raise FileNotFoundError(f"ROS bag file not found: {path}")

    if not path.is_file():
        raise ValueError(f"ROS bag path is not a file: {path}")

    logger.info("Opening bag for reading: %s", path)
    logger.info("Target LiDAR topic: %s", topic)

    # Verify that the topic is available in the bag file
    try:
        bag = rosbag.Bag(str(path), "r")
    except Exception as exc:
        raise ValueError(f"Failed to open ROS bag: {exc}") from exc

    try:
        info = bag.get_type_and_topic_info()
        if topic not in info.topics:
            available_topics = list(info.topics.keys())
            raise ValueError(
                f"Topic '{topic}' not found in the bag file. "
                f"Available topics: {available_topics}"
            )

        # Check message count
        msg_count = info.topics[topic].message_count
        logger.info(
            "Found %d messages in topic %s in bag.",
            msg_count,
            topic,
        )

        if msg_count == 0:
            logger.warning("Topic '%s' has 0 messages.", topic)

        # Read and parse messages sequentially
        for _, msg, _ in bag.read_messages(topics=[topic]):
            try:
                frame = parse_pointcloud_message(msg)
                yield frame
            except (TypeError, ValueError) as exc:
                logger.error(
                    "Skipping message due to parsing error: %s",
                    exc,
                )
                continue
    finally:
        bag.close()
        logger.info("Closed bag: %s", path)
