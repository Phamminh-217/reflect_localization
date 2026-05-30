#!/usr/bin/env python3
"""Script to run Phase 2 & Phase 3 sequentially and verify preprocessing visually.

This script:
1. Loads the configuration threshold_v1.yaml.
2. Reads the ROS bag sequential frames.
3. Preprocesses each frame (NaN removal, range filter, height filter).
4. Saves 2D scatter plots of the raw vs preprocessed points to verify visually.
"""

import argparse
import logging
from pathlib import Path

from rf_threshold.core.preprocessing import preprocess_frame
from rf_threshold.io.bag_reader import read_lidar_frames
from rf_threshold.utils.config import load_yaml_config
from rf_threshold.visualization.plot_frame import plot_frame

# Set up logging format
LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger("run_preprocess_bag")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Verify LiDAR preprocessing and save visualization plots."
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/threshold_v1.yaml",
        help="Path to the YAML configuration file.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="data/results/preprocess_debug",
        help="Directory to save debug images.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=5,
        help="Maximum number of frames to process.",
    )
    return parser.parse_args()


def main() -> None:
    """Run preprocessing verification."""
    args = parse_args()

    # 1. Load config
    config_path = Path(args.config)
    logger.info("Loading config from: %s", config_path)
    try:
        cfg = load_yaml_config(config_path)
    except Exception as exc:
        logger.error("Failed to load config: %s", exc)
        return

    # 2. Extract bag path, topic, and preprocessing config
    try:
        bag_path = cfg["bag"]["path"]
        topic = cfg["bag"]["topic"]
        preprocess_cfg = cfg["preprocessing"]
    except KeyError as exc:
        logger.error("Missing required config key: %s", exc)
        return

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 3. Read, preprocess, and visualize
    logger.info("Reading and preprocessing bag frames...")
    try:
        frame_idx = 0
        for raw_frame in read_lidar_frames(bag_path, topic, cfg):
            # Preprocess the frame
            filtered_frame, summary = preprocess_frame(raw_frame, preprocess_cfg)

            logger.info(
                "[Frame %06d] raw_pts=%d, valid_pts=%d, range_pts=%d, height_pts=%d",
                frame_idx,
                summary["raw_points"],
                summary["valid_points"],
                summary["range_filtered_points"],
                summary["height_filtered_points"],
            )

            # Generate and save a beautiful dark-mode visualization image
            img_path = out_dir / f"frame_{frame_idx:06d}.png"
            plot_frame(
                raw_frame=raw_frame,
                preprocessed_frame=filtered_frame,
                save_path=img_path,
                title_suffix=f"- Preprocessed Frame {frame_idx:06d}",
            )

            frame_idx += 1
            if frame_idx >= args.max_frames:
                break

        logger.info("Successfully completed processing. Debug images saved in %s", out_dir)

    except Exception as exc:
        logger.error("Error during preprocessing run: %s", exc)
        raise


if __name__ == "__main__":
    main()
