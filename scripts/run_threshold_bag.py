#!/usr/bin/env python3
"""Main execution script for threshold-based RF detection on ROS bag files.

This script implements Phase 5 by executing the entire pipeline on a ROS bag
and writing the required reports and visualizations to the output directory.
"""

import argparse
import logging
import shutil
from pathlib import Path
from typing import Any, Dict

import yaml

from rf_threshold.core.detector_pipeline import ThresholdRFDetector
from rf_threshold.io.bag_reader import read_lidar_frames
from rf_threshold.io.result_writer import ResultWriter
from rf_threshold.utils.config import load_yaml_config
from rf_threshold.visualization.plot_frame import plot_frame

# Set up logging format according to CONTRIBUTING.md
LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger("run_threshold_bag")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Run threshold-based RF landmark detection on a ROS bag."
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/threshold_v1.yaml",
        help="Path to the YAML configuration file.",
    )
    parser.add_argument(
        "--bag",
        type=str,
        default=None,
        help="Override path to the ROS bag file.",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default=None,
        help="Override target point cloud topic name.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Override output results directory.",
    )
    return parser.parse_args()


def main() -> None:
    """Run main pipeline execution."""
    args = parse_args()

    # 1. Load config
    config_path = Path(args.config)
    logger.info("Loading configuration from: %s", config_path)
    try:
        cfg = load_yaml_config(config_path)
    except Exception as exc:
        logger.error("Failed to load config: %s", exc)
        return

    # 2. Apply CLI overrides to configuration
    if args.bag is not None:
        cfg["bag"]["path"] = args.bag
    if args.topic is not None:
        cfg["bag"]["topic"] = args.topic
    if args.output is not None:
        cfg["output"]["output_dir"] = args.output

    # Extract primary configs
    bag_path = cfg["bag"]["path"]
    topic = cfg["bag"]["topic"]
    output_dir_str = cfg["output"]["output_dir"]
    output_dir = Path(output_dir_str)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Copy the used configuration file to the output directory as config_used.yaml
    shutil.copy(config_path, output_dir / "config_used.yaml")
    logger.info("Saved copy of config to: %s", output_dir / "config_used.yaml")

    # 3. Instantiate detector and writer
    detector = ThresholdRFDetector(cfg)
    writer = ResultWriter(output_dir)

    save_debug_image = cfg["output"].get("save_debug_image", False)
    debug_images_dir = output_dir / "debug_images"
    if save_debug_image:
        debug_images_dir.mkdir(parents=True, exist_ok=True)

    # 4. Loop through frames
    logger.info("Starting detection pipeline on: %s", bag_path)
    logger.info("Target topic: %s", topic)

    frame_index = 0
    try:
        for raw_frame in read_lidar_frames(bag_path, topic, cfg):
            # Run detection
            valid_detections, debug_data = detector.detect(raw_frame)

            # Log frame progress in the EXACT format required by README.md L548
            # [Frame 000123 | t=12.340]
            # raw=9821 | preprocessed=4210 | bright=64 | clusters=5 | valid=2 | threshold=180.0
            prep_sum = debug_data["preprocessing_summary"]
            print(f"[Frame {frame_index:06d} | t={raw_frame.stamp:.3f}]")
            print(
                f"raw={prep_sum['raw_points']} | "
                f"preprocessed={prep_sum['height_filtered_points']} | "
                f"bright={debug_data['bright_points_count']} | "
                f"clusters={debug_data['num_clusters']} | "
                f"valid={len(valid_detections)} | "
                f"threshold={debug_data['threshold_value']:.1f}"
            )

            # Add results to writer
            writer.add_frame_results(
                frame_index=frame_index,
                stamp=raw_frame.stamp,
                frame_id=raw_frame.frame_id,
                preprocessing_summary=prep_sum,
                threshold=debug_data["threshold_value"],
                bright_points=debug_data["bright_points_count"],
                num_clusters=debug_data["num_clusters"],
                valid_detections=valid_detections,
                rejected_list=debug_data["rejected_clusters"],
            )

            # Save debug image if enabled and we found any RFs
            # Limit the number of saved debug images to avoid filling up disk (max 100 images)
            if save_debug_image and len(valid_detections) > 0 and frame_index < 200:
                img_path = debug_images_dir / f"frame_{frame_index:06d}.png"
                plot_frame(
                    raw_frame=raw_frame,
                    preprocessed_frame=debug_data["preprocessed_frame"],
                    bright_frame=debug_data["bright_frame"],
                    clusters=debug_data["clusters"],
                    detections=valid_detections,
                    save_path=img_path,
                    title_suffix="- RF Detections",
                )

            frame_index += 1

        # 5. Write final files
        writer.write_results()
        logger.info("Pipeline completed successfully. Total processed frames: %d", frame_index)

    except Exception as exc:
        logger.error("Pipeline failed with exception: %s", exc)
        raise


if __name__ == "__main__":
    main()
