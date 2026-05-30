#!/usr/bin/env python3
"""CLI runner script to execute SVD-based RF localization end-to-end offline."""

import argparse
import logging
import sys
from pathlib import Path
import yaml

from rf_threshold.localization.detection_loader import DetectionLoader
from rf_threshold.localization.localizer_pipeline import RFLocalizer
from rf_threshold.localization.fallback_manager import FallbackManager
from rf_threshold.localization.localization_writer import LocalizationWriter

# Setup standard logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("run_svd_localization")


def parse_args() -> argparse.Namespace:
    """Parse command line arguments.

    Returns:
        argparse.Namespace: Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Run offline SVD reflective feature (RF) landmark localization pipeline."
    )
    parser.add_argument(
        "-d",
        "--detections",
        type=str,
        required=True,
        help="Path to detections.json from Phase 1 detector.",
    )
    parser.add_argument(
        "-m",
        "--map",
        type=str,
        required=True,
        help="Path to the global RF landmark map JSON file.",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        required=True,
        help="Path to the system configuration YAML file.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        required=True,
        help="Output directory where poses and debug CSV/JSON files are saved.",
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point for SVD localization runner.

    Returns:
        int: Exit status code (0 for success, non-zero for errors).
    """
    args = parse_args()

    det_path = Path(args.detections)
    map_path = Path(args.map)
    cfg_path = Path(args.config)
    out_dir = Path(args.output)

    # 1. Input existence validation
    if not det_path.exists():
        logger.error("Detections file does not exist: %s", det_path)
        return 1
    if not map_path.exists():
        logger.error("RF map file does not exist: %s", map_path)
        return 1
    if not cfg_path.exists():
        logger.error("Config file does not exist: %s", cfg_path)
        return 1

    # 2. Load YAML configuration
    try:
        with cfg_path.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        if not isinstance(cfg, dict):
            logger.error("Malformed config: Must contain a top-level YAML dictionary.")
            return 1
    except Exception as exc:
        logger.error("Failed to parse config file: %s", exc)
        return 1

    # 3. Initialize Loader, Localizer, Fallback, and Writer
    logger.info("Initializing offline SVD localization runner...")
    try:
        loader = DetectionLoader(det_path)
        localizer = RFLocalizer(str(map_path), cfg)
        fallback_mgr = FallbackManager(cfg)
        writer = LocalizationWriter(out_dir)
    except Exception as exc:
        logger.error("Initialization failure: %s", exc)
        return 1

    # 4. Load detections
    logger.info("Loading detection frames from %s...", det_path)
    try:
        frames = loader.load_frames()
        logger.info("Loaded %d detection frames.", len(frames))
    except Exception as exc:
        logger.error("Failed to load detection frames: %s", exc)
        return 1

    # 5. Execute core pipeline frame-by-frame loop
    logger.info("Running frame-by-frame localization loop...")
    results = []
    fallback_outputs = []
    frame_indices = []
    num_detections_list = []

    for frame in frames:
        try:
            # SVD-based localization
            res = localizer.localize(frame.detections, frame.stamp)
            # Apply Fallback continuity manager
            out = fallback_mgr.handle_result(frame.frame_index, frame.stamp, res)

            results.append(res)
            fallback_outputs.append(out)
            frame_indices.append(frame.frame_index)
            num_detections_list.append(len(frame.detections))
        except Exception as exc:
            logger.error(
                "Unexpected pipeline exception at frame %d: %s",
                frame.frame_index,
                exc,
            )
            return 1

    # 6. Write all 8 SRE debug evidence files
    logger.info("Writing all 8 debug evidence outputs to %s...", out_dir)
    try:
        writer.write_results(results, fallback_outputs, frame_indices, num_detections_list)
        logger.info("End-to-End SVD Localization completed successfully.")
    except Exception as exc:
        logger.error("Failed to write results: %s", exc)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
