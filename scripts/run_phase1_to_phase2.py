#!/usr/bin/env python3
"""Coordinator script to execute both Phase 1 (detector) and Phase 2 (localizer) end-to-end."""

import argparse
import logging
import subprocess
import sys
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("run_phase1_to_phase2")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Phase 1 RF Detection and Phase 2 SVD Localization end-to-end."
    )
    parser.add_argument(
        "-b",
        "--bag",
        type=str,
        default=None,
        help="Path to the ROS bag file (required unless --phase2-only is specified).",
    )
    parser.add_argument(
        "-m",
        "--map",
        type=str,
        required=True,
        help="Path to the global RF map JSON file.",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default="config/threshold_field_v1.yaml",
        help="Path to the system configuration YAML file.",
    )
    parser.add_argument(
        "-n",
        "--run-name",
        type=str,
        required=True,
        help="Run name for naming output directories (e.g. run_001).",
    )
    parser.add_argument(
        "--phase1-only",
        action="store_true",
        help="Only run Phase 1 (detection) and exit.",
    )
    parser.add_argument(
        "--phase2-only",
        action="store_true",
        help="Skip Phase 1 (detection) and run Phase 2 (localization) using existing detections.json.",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose output.",
    )
    return parser.parse_args()


import os

def run_command(cmd: list, description: str) -> bool:
    logger.info("Executing: %s", " ".join(cmd))
    env = os.environ.copy()
    src_dir = str(Path(__file__).parent.parent / "src")
    if "PYTHONPATH" in env:
        env["PYTHONPATH"] = src_dir + os.pathsep + env["PYTHONPATH"]
    else:
        env["PYTHONPATH"] = src_dir
        
    try:
        subprocess.run(cmd, check=True, env=env)
        logger.info("%s completed successfully.", description)
        return True
    except subprocess.CalledProcessError as exc:
        logger.error("%s failed: %s", description, exc)
        return False


def main() -> int:
    args = parse_args()

    # Define paths based on run-name
    run_dir = Path("data/results") / args.run_name
    detections_file = run_dir / "detections.json"
    localization_dir = run_dir / "localization"
    report_file = run_dir / "validation_report.md"

    # Step 1: Run Phase 1 Detection
    if not args.phase2_only:
        if not args.bag:
            logger.error("--bag is required unless --phase2-only is specified.")
            return 1
        
        logger.info("=== Starting Phase 1: RF Detection ===")
        phase1_cmd = [
            sys.executable,
            "scripts/run_threshold_bag.py",
            "--config", args.config,
            "--bag", args.bag,
            "--output", str(run_dir),
        ]
        if not run_command(phase1_cmd, "Phase 1 RF Detection"):
            return 1

    # Check that detections.json was indeed created or exists
    if not detections_file.exists():
        logger.error("Detections file not found: %s", detections_file)
        return 1

    if args.phase1_only:
        logger.info("Phase 1 finished. Stopping as requested by --phase1-only.")
        return 0

    # Step 2: Run Phase 2 SVD Localization
    logger.info("=== Starting Phase 2: SVD Localization ===")
    phase2_cmd = [
        sys.executable,
        "scripts/run_svd_localization.py",
        "--detections", str(detections_file),
        "--map", args.map,
        "--config", args.config,
        "--run-name", args.run_name,
    ]
    if args.verbose:
        phase2_cmd.append("-v")

    if not run_command(phase2_cmd, "Phase 2 SVD Localization"):
        return 1

    # Step 3: Run Validation Report & Plot Generator
    logger.info("=== Starting Validation Report Generator ===")
    report_cmd = [
        sys.executable,
        "scripts/generate_validation_report.py",
        "--results-dir", str(localization_dir),
        "--map", args.map,
        "--output", str(report_file),
    ]
    if not run_command(report_cmd, "Validation Report Generation"):
        return 1

    logger.info("=== End-to-End Run '%s' Completed successfully ===", args.run_name)
    logger.info("Validation report saved to: %s", report_file)
    return 0


if __name__ == "__main__":
    sys.exit(main())
