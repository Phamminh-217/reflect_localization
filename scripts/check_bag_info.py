#!/usr/bin/env python3
"""Inspect basic information of a ROS bag file.

This script is used before implementing the LiDAR parser. It helps verify:
- Whether the bag file exists.
- Which topics are available.
- Which message types are stored.
- How many messages each topic contains.
- Bag start/end time and duration.
"""

import argparse
import logging
from pathlib import Path
from typing import Dict, Tuple
import rosbag


LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger("check_bag_info")


def inspect_bag(bag_path: Path) -> None:
    """Inspect and print ROS bag metadata.

    Args:
        bag_path: Path to the ROS bag file.

    Raises:
        FileNotFoundError: If the bag file does not exist.
        ValueError: If the path is not a file.
    """
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag file not found: {bag_path}")

    if not bag_path.is_file():
        raise ValueError(f"Bag path is not a file: {bag_path}")

    logger.info("Opening bag: %s", bag_path)

    with rosbag.Bag(str(bag_path), "r") as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        duration = end_time - start_time
        message_count = bag.get_message_count()

        logger.info("Bag start time : %.6f", start_time)
        logger.info("Bag end time   : %.6f", end_time)
        logger.info("Duration       : %.3f seconds", duration)
        logger.info("Total messages : %d", message_count)

        topic_info: Dict[str, Tuple[str, int]] = bag.get_type_and_topic_info().topics

        print("\nAvailable topics:")
        print("-" * 90)
        print(f"{'Topic':45s} {'Message Type':35s} {'Count':>8s}")
        print("-" * 90)

        for topic_name, info in sorted(topic_info.items()):
            msg_type = info.msg_type
            count = info.message_count
            print(f"{topic_name:45s} {msg_type:35s} {count:8d}")

        print("-" * 90)

        lidar_candidates = [
            topic_name
            for topic_name, info in topic_info.items()
            if "PointCloud2" in info.msg_type
            or "livox" in topic_name.lower()
            or "lidar" in topic_name.lower()
        ]

        if lidar_candidates:
            print("\nPossible LiDAR topics:")
            for topic_name in lidar_candidates:
                print(f"  - {topic_name}")
        else:
            print("\n[WARNING] No obvious LiDAR topic found.")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Inspect ROS bag topics and message types."
    )

    parser.add_argument(
        "--bag",
        required=True,
        type=str,
        help="Path to the ROS bag file.",
    )

    return parser.parse_args()


def main() -> None:
    """Run bag inspection."""
    args = parse_args()
    bag_path = Path(args.bag)

    inspect_bag(bag_path)


if __name__ == "__main__":
    main()
