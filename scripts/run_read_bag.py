#!/usr/bin/env python3
"""Script to verify reading ROS bag and parsing LiDAR data.

This is a Phase 2 entry-point script to verify that:
1. The bag file can be opened.
2. The point cloud parser can parse PointCloud2 messages.
3. The LidarFrame objects are correctly created.
"""

import argparse
import logging
from pathlib import Path

import numpy as np

from rf_threshold.io.bag_reader import read_lidar_frames
from rf_threshold.utils.config import load_yaml_config

# Set up logging format according to CONTRIBUTING.md
LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger("run_read_bag")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Verify reading ROS bag and parsing LiDAR data."
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/threshold_v1.yaml",
        help="Path to the YAML configuration file.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=10,
        help="Maximum number of frames to read and print.",
    )
    return parser.parse_args()


def main() -> None:
    """Run verification script."""
    args = parse_args()

    # 1. Load configuration
    config_path = Path(args.config)
    logger.info("Loading config from: %s", config_path)
    try:
        cfg = load_yaml_config(config_path)
    except Exception as exc:
        logger.error("Failed to load config: %s", exc)
        return

    # 2. Extract bag path and topic
    try:
        bag_path = cfg["bag"]["path"]
        topic = cfg["bag"]["topic"]
    except KeyError as exc:
        logger.error("Missing required config key: %s", exc)
        return

    logger.info("Reading bag: %s", bag_path)
    logger.info("Topic: %s", topic)

    # 3. Read and print frames
    try:
        frame_idx = 0
        for frame in read_lidar_frames(bag_path, topic, cfg):
            # Calculate intensity stats
            intensity = frame.intensity
            if intensity.size > 0:
                min_i = np.min(intensity)
                max_i = np.max(intensity)
                mean_i = np.mean(intensity)
            else:
                min_i = max_i = mean_i = 0.0

            print(
                f"[Frame {frame_idx:06d}] stamp={frame.stamp:.3f} | "
                f"frame_id={frame.frame_id} | "
                f"points={frame.points_xyz.shape[0]} | "
                f"intensity=[{min_i:.1f}, {max_i:.1f}], mean={mean_i:.1f}"
            )

            frame_idx += 1
            if frame_idx >= args.max_frames:
                logger.info(
                    "Reached maximum frame limit of %d. Stopping.",
                    args.max_frames,
                )
                break

    except Exception as exc:
        logger.error("Error during bag reading: %s", exc)
        raise


if __name__ == "__main__":
    main()
