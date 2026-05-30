"""Visualization utilities for LiDAR frames and RF detections."""

import logging
from pathlib import Path
from typing import Any, List, Optional, Union

import matplotlib.pyplot as plt
import numpy as np

from rf_threshold.core.frame import LidarFrame, RFCluster, RFDetection

logger = logging.getLogger("plot_frame")


def plot_frame(
    raw_frame: Optional[LidarFrame] = None,
    preprocessed_frame: Optional[LidarFrame] = None,
    bright_frame: Optional[LidarFrame] = None,
    clusters: Optional[List[RFCluster]] = None,
    detections: Optional[List[RFDetection]] = None,
    save_path: Optional[Union[str, Path]] = None,
    show: bool = False,
    title_suffix: str = "",
) -> None:
    """Create a premium 2D scatter plot visualization of a LiDAR frame.

    This function handles optional layers to support progressive pipeline stages:
    1. Preprocessed points (soft blue-gray background)
    2. Bright points (vibrant orange highlight)
    3. Valid clusters (cyan point groups + dashed bounding boxes)
    4. Estimated RF centers (neon green star with halo)

    Args:
        raw_frame: Optional original thuy LiDAR frame.
        preprocessed_frame: Optional filtered LidarFrame.
        bright_frame: Optional LidarFrame containing points above threshold.
        clusters: Optional list of validated RFCluster candidates.
        detections: Optional list of validated RFDetection landmarks.
        save_path: File path to save the generated image.
        show: If True, display the plot interactively.
        title_suffix: Additional text to append to the plot title.
    """
    # Create figure with dark theme style (tailored HSL colors)
    # Background slate-950: '#020617', Slate-900 panel: '#0f172a'
    fig, ax = plt.subplots(figsize=(10, 10), facecolor="#020617")
    ax.set_facecolor("#0a0f1d")

    # Force equal scaling for coordinates
    ax.set_aspect("equal", adjustable="box")

    # Premium grid styling
    ax.grid(True, color="#1e293b", linestyle="--", linewidth=0.5, alpha=0.8)

    # Track coordinate ranges to set proper axis limits
    x_min, x_max = -8.0, 8.0
    y_min, y_max = -8.0, 8.0

    # 1. Plot Preprocessed / Raw Point Cloud
    # Use preprocessed_frame if available, otherwise fallback to raw_frame
    bg_frame = preprocessed_frame if preprocessed_frame is not None else raw_frame
    if bg_frame is not None and bg_frame.points_xyz.shape[0] > 0:
        xyz = bg_frame.points_xyz
        ax.scatter(
            xyz[:, 0],
            xyz[:, 1],
            s=1.0,
            color="#334155",  # Slate-700
            alpha=0.4,
            label=f"Point Cloud ({xyz.shape[0]} pts)",
            edgecolors="none",
        )
        # Update limits dynamically if coordinates exceed default bounds
        x_min = min(x_min, np.min(xyz[:, 0]) - 0.5)
        x_max = max(x_max, np.max(xyz[:, 0]) + 0.5)
        y_min = min(y_min, np.min(xyz[:, 1]) - 0.5)
        y_max = max(y_max, np.max(xyz[:, 1]) + 0.5)

    # 2. Plot Bright Points (intensity above threshold)
    if bright_frame is not None and bright_frame.points_xyz.shape[0] > 0:
        xyz_bright = bright_frame.points_xyz
        ax.scatter(
            xyz_bright[:, 0],
            xyz_bright[:, 1],
            s=4.0,
            color="#f97316",  # Amber-500
            alpha=0.8,
            label=f"Bright Points ({xyz_bright.shape[0]} pts)",
            edgecolors="none",
        )

    # 3. Plot Clusters (cyan points + dashed bounding boxes)
    if clusters is not None and len(clusters) > 0:
        for idx, cluster in enumerate(clusters):
            xyz_cl = cluster.points_xyz
            # Draw cluster points
            ax.scatter(
                xyz_cl[:, 0],
                xyz_cl[:, 1],
                s=8.0,
                color="#06b6d4",  # Cyan-500
                alpha=0.9,
                label="RF Clusters" if idx == 0 else "",
                edgecolors="none",
            )
            # Draw bounding box
            if xyz_cl.shape[0] >= 2:
                c_min = np.min(xyz_cl, axis=0)
                c_max = np.max(xyz_cl, axis=0)
                rect = plt.Rectangle(
                    (c_min[0], c_min[1]),
                    c_max[0] - c_min[0],
                    c_max[1] - c_min[1],
                    fill=False,
                    edgecolor="#22d3ee",
                    linestyle="--",
                    linewidth=0.8,
                    alpha=0.6,
                )
                ax.add_patch(rect)

    # 4. Plot RF Detections / Centers (Neon green star with outer ring)
    if detections is not None and len(detections) > 0:
        for idx, det in enumerate(detections):
            center = det.center_lidar
            # Draw outer ring (halo effect)
            ax.scatter(
                center[0],
                center[1],
                s=200,
                facecolors="none",
                edgecolors="#4ade80",  # Emerald-400
                alpha=0.4,
                linewidths=1.0,
            )
            # Draw center star
            ax.scatter(
                center[0],
                center[1],
                s=100,
                color="#22c55e",  # Green-500
                marker="*",
                label="RF Center" if idx == 0 else "",
                edgecolors="white",
                linewidths=0.5,
            )
            # Annotate ID and Intensity
            ax.annotate(
                f"RF {det.detection_id} (I={det.mean_intensity:.1f})",
                (center[0], center[1]),
                textcoords="offset points",
                xytext=(10, 10),
                ha="left",
                color="#f8fafc",  # Slate-50
                fontsize=8,
                weight="bold",
                bbox=dict(
                    boxstyle="round,pad=0.2",
                    fc="#0f172a",
                    ec="#334155",
                    alpha=0.8,
                ),
            )

    # Style axes and titles
    stamp_str = ""
    if bg_frame is not None:
        stamp_str = f" | t={bg_frame.stamp:.3f}s"

    ax.set_title(
        f"RF Threshold Localization {title_suffix}{stamp_str}",
        color="#f8fafc",
        fontsize=12,
        pad=15,
        weight="bold",
    )
    ax.set_xlabel("X (LiDAR coordinate / meters)", color="#94a3b8", labelpad=10)
    ax.set_ylabel("Y (LiDAR coordinate / meters)", color="#94a3b8", labelpad=10)

    # Limit coordinate view
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    # Style ticks
    ax.tick_params(colors="#64748b", which="both", labelsize=9)
    for spine in ax.spines.values():
        spine.set_color("#1e293b")

    # Legend with transparent premium box styling
    legend = ax.legend(
        loc="upper right",
        facecolor="#0f172a",
        edgecolor="#334155",
        labelcolor="#f8fafc",
        fontsize=9,
        shadow=True,
    )
    if legend:
        legend.get_frame().set_alpha(0.85)

    # Add a watermark-like coordinate system note
    ax.text(
        0.02,
        0.02,
        "Livox Mid-360 [lidar_frame]",
        transform=ax.transAxes,
        color="#475569",
        fontsize=8,
        ha="left",
        va="bottom",
    )

    plt.tight_layout()

    # Save to file if path is specified
    if save_path is not None:
        out_path = Path(save_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=150, facecolor=fig.get_facecolor(), edgecolor="none")
        logger.info("Saved debug visualization image to: %s", out_path)

    if show:
        plt.show()

    plt.close(fig)
