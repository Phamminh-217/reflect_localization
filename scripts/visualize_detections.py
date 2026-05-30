#!/usr/bin/env python3
"""Script to read detections.json and visualize detected RF center trajectories."""

import argparse
import json
import logging
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Set up logging format
LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger("visualize_detections")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="Summarize and visualize RF detection trajectories."
    )
    parser.add_argument(
        "--result",
        type=str,
        default="data/results/sample_run/detections.json",
        help="Path to the detections.json results file.",
    )
    parser.add_argument(
        "--save-plot",
        type=str,
        default="data/results/sample_run/rf_trajectory.png",
        help="Path to save the generated summary trajectory plot.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    result_path = Path(args.result)
    if not result_path.exists():
        logger.error("Result JSON file not found: %s", result_path)
        return

    logger.info("Reading detections from: %s", result_path)
    with result_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    detections = data.get("detections", [])
    total_frames = len(detections)
    logger.info("Total frames parsed: %d", total_frames)

    # Accumulate all detected centers
    all_centers = []
    all_scores = []
    all_intensities = []
    rf_counts_per_frame = []

    for frame in detections:
        num_valid = frame.get("num_valid_detections", 0)
        rf_counts_per_frame.append(num_valid)

        for obj in frame.get("objects", []):
            all_centers.append(obj["center_lidar"])
            all_scores.append(obj["score"])
            all_intensities.append(obj["mean_intensity"])

    if len(all_centers) == 0:
        logger.warning("No RF landmarks were detected in the entire bag! Plot cannot be generated.")
        return

    centers_arr = np.array(all_centers)
    scores_arr = np.array(all_scores)
    intensities_arr = np.array(all_intensities)
    counts_arr = np.array(rf_counts_per_frame)

    # Print summary statistics
    logger.info("=" * 60)
    logger.info("THỐNG KÊ KẾT QUẢ PHÁT HIỆN RF:")
    logger.info("  Tổng số frame đã xử lý        : %d", total_frames)
    logger.info("  Tổng số RF phát hiện được     : %d", len(centers_arr))
    logger.info("  Số RF trung bình mỗi frame    : %.2f", np.mean(counts_arr))
    logger.info("  Tỷ lệ frame có RF (Recall rate): %.1f%%", np.sum(counts_arr > 0) / total_frames * 100)
    logger.info("  Số mốc RF tối đa trong 1 frame : %d", np.max(counts_arr))
    logger.info("  Cường độ trung bình của RF    : %.1f (dải [%.1f, %.1f])", np.mean(intensities_arr), np.min(intensities_arr), np.max(intensities_arr))
    logger.info("  Điểm tin cậy trung bình (Score): %.3f (dải [%.3f, %.3f])", np.mean(scores_arr), np.min(scores_arr), np.max(scores_arr))
    logger.info("=" * 60)

    # Generate a beautiful summary trajectory plot (dark mode style)
    fig, ax = plt.subplots(figsize=(10, 8), facecolor="#020617")
    ax.set_facecolor("#0a0f1d")

    ax.grid(True, color="#1e293b", linestyle="--", linewidth=0.5, alpha=0.8)

    # Scatter plot of all detected centers in 2D (colored by intensity)
    sc = ax.scatter(
        centers_arr[:, 0],
        centers_arr[:, 1],
        c=intensities_arr,
        cmap="plasma",
        s=15,
        alpha=0.6,
        edgecolors="none",
    )

    # Add a glowing star at the average positions of major clusters to represent matched landmark positions
    # We can perform a quick DBSCAN on all detected centers to find the permanent RF locations in the map!
    from sklearn.cluster import DBSCAN
    db = DBSCAN(eps=0.15, min_samples=5).fit(centers_arr[:, :2])
    labels = db.labels_
    unique_labels = set(labels)

    landmark_count = 0
    for label in unique_labels:
        if label == -1:
            continue
        landmark_pts = centers_arr[labels == label]
        landmark_center = np.mean(landmark_pts, axis=0)
        # Draw outer ring
        ax.scatter(
            landmark_center[0],
            landmark_center[1],
            s=300,
            facecolors="none",
            edgecolors="#22c55e",
            linewidths=1.5,
            alpha=0.8,
        )
        # Draw star
        ax.scatter(
            landmark_center[0],
            landmark_center[1],
            s=120,
            color="#22c55e",
            marker="*",
            edgecolors="white",
            linewidths=0.5,
            label="Ước lượng mốc bản đồ" if landmark_count == 0 else "",
        )
        # Annotate
        ax.annotate(
            f"Landmark {landmark_count}\n(X={landmark_center[0]:.2f}, Y={landmark_center[1]:.2f})",
            (landmark_center[0], landmark_center[1]),
            textcoords="offset points",
            xytext=(15, -5),
            color="#f8fafc",
            fontsize=8,
            bbox=dict(
                boxstyle="round,pad=0.2",
                fc="#1e293b",
                ec="#334155",
                alpha=0.9,
            ),
        )
        landmark_count += 1

    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("Cường độ phản xạ trung bình (Mean Intensity)", color="#94a3b8", labelpad=10)
    cbar.ax.yaxis.set_tick_params(color="#64748b", labelcolor="#64748b")
    cbar.solids.set_edgecolor("none")

    ax.set_title("Quỹ đạo tâm RF được phát hiện trong hệ tọa độ LiDAR (2D)", color="#f8fafc", fontsize=12, pad=15, weight="bold")
    ax.set_xlabel("X (m)", color="#94a3b8")
    ax.set_ylabel("Y (m)", color="#94a3b8")

    ax.tick_params(colors="#64748b", labelsize=9)
    for spine in ax.spines.values():
        spine.set_color("#1e293b")

    plt.axis("equal")
    plt.tight_layout()

    save_path = Path(args.save_plot)
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=150, facecolor=fig.get_facecolor(), edgecolor="none")
    logger.info("Saved trajectory plot to: %s", save_path)
    plt.close(fig)


if __name__ == "__main__":
    main()
