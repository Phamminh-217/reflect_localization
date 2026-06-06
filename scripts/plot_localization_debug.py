#!/usr/bin/env python3
"""Phase 2.6.4 — Debug Plot Script for SVD Localization.

Reads all 8 debug CSV/JSON files written by LocalizationWriter and produces
diagnostic plots covering:
  1. Robot trajectory (OK vs Fallback vs Rejected).
  2. Residual RMSE per frame.
  3. Geometry debug (spread, condition number, warnings).
  4. Frame-level status breakdown (stacked bar summary).
  5. RF map landmarks + matched pairs overlay for a single selected frame.

Usage:
    python scripts/plot_localization_debug.py \\
        --output <localization_output_dir> \\
        --map <path/to/rf_map_v1.json> \\
        [--frame <frame_index>] \\
        [--save <output_dir_for_plots>]

Dependencies:
    pip install matplotlib numpy
"""

import argparse
import csv
import json
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger("plot_localization_debug")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

# --------------------------------------------------------------------------- #
# CSV / JSON Helpers                                                            #
# --------------------------------------------------------------------------- #


def _load_csv(path: Path) -> List[Dict[str, str]]:
    """Load a CSV file and return a list of row dicts."""
    if not path.exists():
        raise FileNotFoundError(f"Required file not found: {path}")
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _safe_float(val: str) -> Optional[float]:
    """Return float or None for empty/invalid string."""
    try:
        return float(val)
    except (TypeError, ValueError):
        return None


def _load_rf_map(map_path: Path) -> List[Dict[str, Any]]:
    """Load RF map JSON and return list of landmark dicts."""
    with map_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return data.get("landmarks", [])


# --------------------------------------------------------------------------- #
# Data Extraction                                                               #
# --------------------------------------------------------------------------- #


def _extract_poses(
    rows: List[Dict[str, str]],
) -> Tuple[List[int], List[float], List[float], List[float], List[str], List[bool]]:
    """Extract frame_index, x, y, yaw, status, is_fallback from poses.csv rows."""
    frame_indices: List[int] = []
    xs: List[float] = []
    ys: List[float] = []
    yaws: List[float] = []
    statuses: List[str] = []
    is_fallbacks: List[bool] = []

    for row in rows:
        x = _safe_float(row.get("x", ""))
        y = _safe_float(row.get("y", ""))
        yaw = _safe_float(row.get("yaw", ""))
        if x is None or y is None or yaw is None:
            continue  # Skip frames with no pose (pure rejected)
        frame_indices.append(int(row["frame_index"]))
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
        statuses.append(row.get("status", ""))
        is_fallbacks.append(row.get("is_fallback", "false").lower() == "true")

    return frame_indices, xs, ys, yaws, statuses, is_fallbacks


def _extract_residuals(rows: List[Dict[str, str]]) -> Tuple[List[int], List[float]]:
    """Extract frame_index and residual_rmse from poses.csv rows."""
    fidxs: List[int] = []
    resids: List[float] = []
    for row in rows:
        r = _safe_float(row.get("residual_rmse", ""))
        if r is not None:
            fidxs.append(int(row["frame_index"]))
            resids.append(r)
    return fidxs, resids


def _extract_geometry(
    rows: List[Dict[str, str]],
) -> Tuple[List[int], List[Optional[float]], List[Optional[float]], List[str]]:
    """Extract geometry debug data from geometry_debug.csv rows."""
    fidxs: List[int] = []
    spreads: List[Optional[float]] = []
    conds: List[Optional[float]] = []
    warnings: List[str] = []
    for row in rows:
        fidxs.append(int(row["frame_index"]))
        spreads.append(_safe_float(row.get("spread_lidar", "")))
        conds.append(_safe_float(row.get("condition_number_lidar", "")))
        warnings.append(row.get("warning", ""))
    return fidxs, spreads, conds, warnings


def _extract_frame_status(rows: List[Dict[str, str]]) -> Dict[str, int]:
    """Tally status counts from frame_debug.csv."""
    counts: Dict[str, int] = {}
    for row in rows:
        status = row.get("status", "UNKNOWN")
        counts[status] = counts.get(status, 0) + 1
    return counts


def _extract_association_for_frame(
    rows: List[Dict[str, str]], frame_index: int
) -> Tuple[List[float], List[float], List[float], List[float]]:
    """Return (x_lidar, y_lidar, x_map, y_map) for a specific frame from association_debug.csv."""
    xl, yl, xm, ym = [], [], [], []
    for row in rows:
        if int(row["frame_index"]) != frame_index:
            continue
        lx = _safe_float(row.get("x_lidar", ""))
        ly = _safe_float(row.get("y_lidar", ""))
        mx = _safe_float(row.get("x_map", ""))
        my = _safe_float(row.get("y_map", ""))
        if None in (lx, ly, mx, my):
            continue
        xl.append(lx)
        yl.append(ly)
        xm.append(mx)
        ym.append(my)
    return xl, yl, xm, ym


# --------------------------------------------------------------------------- #
# Plotting                                                                      #
# --------------------------------------------------------------------------- #


def _plot_trajectory(
    frame_indices: List[int],
    xs: List[float],
    ys: List[float],
    yaws: List[float],
    is_fallbacks: List[bool],
    save_path: Optional[Path],
) -> None:
    """Plot 1 — Robot Trajectory: OK (green), Fallback (orange) with yaw arrows."""
    import matplotlib.pyplot as plt
    import numpy as np

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title("Robot Trajectory (OK vs Fallback)", fontsize=14, fontweight="bold")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.35)

    arrow_len = 0.12

    for i, (fi, x, y, yaw, is_fb) in enumerate(
        zip(frame_indices, xs, ys, yaws, is_fallbacks)
    ):
        color = "#F5A623" if is_fb else "#27AE60"
        marker = "s" if is_fb else "o"
        
        # Plot markers, arrows, and labels at a stride of 20 to keep it thin and clean,
        # but always plot fallback frames, first frame, and last frame.
        if is_fb or (i % 20 == 0) or (i == 0) or (i == len(xs) - 1):
            ax.plot(x, y, marker=marker, color=color, markersize=4, zorder=3)
            # Yaw direction arrow
            dx = arrow_len * np.cos(yaw)
            dy = arrow_len * np.sin(yaw)
            ax.annotate(
                "",
                xy=(x + dx, y + dy),
                xytext=(x, y),
                arrowprops=dict(arrowstyle="->", color=color, lw=1.0),
                zorder=4,
            )
            # Frame index label
            ax.annotate(
                str(fi),
                xy=(x, y),
                xytext=(3, 3),
                textcoords="offset points",
                fontsize=6,
                color="#555555",
                alpha=0.8,
            )

    # Draw trajectory line
    if len(xs) >= 2:
        ax.plot(xs, ys, "-", color="#3498DB", linewidth=0.8, zorder=2)

    # Legend
    from matplotlib.lines import Line2D

    legend_elements = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#27AE60",
               markersize=10, label="OK"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor="#F5A623",
               markersize=10, label="Fallback"),
    ]
    ax.legend(handles=legend_elements, loc="best")

    plt.tight_layout()
    _save_or_show(fig, save_path, "01_trajectory.png")


def _plot_residuals(
    frame_indices: List[int],
    residuals: List[float],
    max_residual_threshold: float,
    save_path: Optional[Path],
) -> None:
    """Plot 2 — Residual RMSE per frame."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_title("SVD Residual RMSE per Frame", fontsize=14, fontweight="bold")
    ax.set_xlabel("Frame Index")
    ax.set_ylabel("Residual RMSE [m]")
    ax.grid(True, alpha=0.35)

    colors = ["#E74C3C" if r > max_residual_threshold else "#2E86C1" for r in residuals]
    ax.bar(frame_indices, residuals, color=colors, edgecolor="white", linewidth=0.5)
    ax.axhline(
        y=max_residual_threshold,
        color="#E74C3C",
        linestyle="--",
        linewidth=1.5,
        label=f"Threshold ({max_residual_threshold:.3f} m)",
    )
    ax.legend()

    plt.tight_layout()
    _save_or_show(fig, save_path, "02_residuals.png")


def _plot_geometry(
    frame_indices: List[int],
    spreads: List[Optional[float]],
    conds: List[Optional[float]],
    warnings: List[str],
    save_path: Optional[Path],
) -> None:
    """Plot 3 — Geometry debug: spread and condition number with NEAR_COLLINEAR highlights."""
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle("Geometry Debug per Frame", fontsize=14, fontweight="bold")

    # Spread
    ax1.set_ylabel("Spatial Spread [m]")
    ax1.grid(True, alpha=0.35)
    spread_vals = [s if s is not None else 0.0 for s in spreads]
    ax1.bar(frame_indices, spread_vals, color="#2ECC71", edgecolor="white", linewidth=0.5, label="spread_lidar")
    for fi, w in zip(frame_indices, warnings):
        if w == "NEAR_COLLINEAR":
            ax1.axvspan(fi - 0.5, fi + 0.5, alpha=0.25, color="#E67E22", label="_")
    ax1.legend(["spread_lidar"])

    # Condition number
    ax2.set_xlabel("Frame Index")
    ax2.set_ylabel("Condition Number")
    ax2.grid(True, alpha=0.35)
    cond_vals = [c if c is not None else 0.0 for c in conds]
    ax2.bar(frame_indices, cond_vals, color="#3498DB", edgecolor="white", linewidth=0.5, label="cond_lidar")
    for fi, w in zip(frame_indices, warnings):
        if w == "NEAR_COLLINEAR":
            ax2.axvspan(fi - 0.5, fi + 0.5, alpha=0.25, color="#E67E22")
    ax2.legend(["cond_lidar", "NEAR_COLLINEAR zone"])

    plt.tight_layout()
    _save_or_show(fig, save_path, "03_geometry.png")


def _plot_status_summary(
    status_counts: Dict[str, int],
    save_path: Optional[Path],
) -> None:
    """Plot 4 — Pie / bar chart of frame status breakdown."""
    import matplotlib.pyplot as plt

    STATUS_COLORS = {
        "OK": "#27AE60",
        "FALLBACK_LAST_VALID_POSE": "#F5A623",
        "INSUFFICIENT_DETECTIONS": "#E74C3C",
        "ASSOCIATION_FAILED": "#9B59B6",
        "DEGENERATE_GEOMETRY": "#E67E22",
        "HIGH_RESIDUAL": "#C0392B",
        "ERROR": "#7F8C8D",
    }

    labels = list(status_counts.keys())
    values = [status_counts[l] for l in labels]
    colors = [STATUS_COLORS.get(l, "#95A5A6") for l in labels]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title("Frame Status Breakdown", fontsize=14, fontweight="bold")
    wedges, texts, autotexts = ax.pie(
        values,
        labels=labels,
        autopct="%1.0f%%",
        colors=colors,
        startangle=140,
        pctdistance=0.78,
    )
    for t in texts:
        t.set_fontsize(9)
    for at in autotexts:
        at.set_fontsize(8)
        at.set_color("white")
        at.set_fontweight("bold")

    plt.tight_layout()
    _save_or_show(fig, save_path, "04_status_summary.png")


def _plot_frame_association(
    frame_index: int,
    xl: List[float],
    yl: List[float],
    xm: List[float],
    ym: List[float],
    landmarks: List[Dict[str, Any]],
    poses_rows: List[Dict[str, str]],
    save_path: Optional[Path],
) -> None:
    """Plot 5 — RF map + detected observations + matched pairs for one frame."""
    import matplotlib.pyplot as plt
    import numpy as np

    # Retrieve pose for this frame (to draw robot arrow)
    pose_row = next((r for r in poses_rows if int(r["frame_index"]) == frame_index), None)

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.set_title(
        f"Frame {frame_index} — Association Overlay", fontsize=14, fontweight="bold"
    )
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.35)

    # Draw all map landmarks (background)
    for lm in landmarks:
        pos = lm.get("position_map", [0, 0, 0])
        lid = lm.get("id", "?")
        ax.plot(pos[0], pos[1], "^", color="#BDC3C7", markersize=14, zorder=1)
        ax.annotate(
            f"L{lid}",
            xy=(pos[0], pos[1]),
            xytext=(4, 6),
            textcoords="offset points",
            fontsize=8,
            color="#7F8C8D",
        )

    # Draw matched pairs with connection lines
    for i, (lx, ly, mx, my) in enumerate(zip(xl, yl, xm, ym)):
        # Map landmark (matched)
        ax.plot(mx, my, "^", color="#2E86C1", markersize=14, zorder=3,
                label="Map landmark (matched)" if i == 0 else "")
        # Lidar detection
        ax.plot(lx, ly, "o", color="#E74C3C", markersize=10, zorder=4,
                label="LiDAR detection" if i == 0 else "")
        # Connecting line
        ax.plot([lx, mx], [ly, my], "--", color="#AAB7B8", linewidth=1.2, zorder=2)

    # Draw estimated robot pose if available
    if pose_row is not None:
        px = _safe_float(pose_row.get("x", ""))
        py = _safe_float(pose_row.get("y", ""))
        pyaw = _safe_float(pose_row.get("yaw", ""))
        is_fb = pose_row.get("is_fallback", "false").lower() == "true"
        if px is not None and py is not None and pyaw is not None:
            robot_color = "#F5A623" if is_fb else "#27AE60"
            ax.plot(px, py, "*", color=robot_color, markersize=18, zorder=5,
                    label=f"Robot pose ({'Fallback' if is_fb else 'OK'})")
            arr_len = 0.15
            dx = arr_len * np.cos(pyaw)
            dy = arr_len * np.sin(pyaw)
            ax.annotate(
                "",
                xy=(px + dx, py + dy),
                xytext=(px, py),
                arrowprops=dict(arrowstyle="->", color=robot_color, lw=2.5),
                zorder=6,
            )

    ax.legend(loc="best", fontsize=9)
    plt.tight_layout()
    _save_or_show(fig, save_path, f"05_frame_{frame_index}_association.png")


# --------------------------------------------------------------------------- #
# Save / Show Helper                                                            #
# --------------------------------------------------------------------------- #


def _save_or_show(fig: Any, save_dir: Optional[Path], filename: str) -> None:
    """Save figure to disk if save_dir given, else show interactively."""
    import matplotlib.pyplot as plt

    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)
        out = save_dir / filename
        fig.savefig(out, dpi=150, bbox_inches="tight")
        logger.info("Saved plot: %s", out)
    else:
        plt.show()
    plt.close(fig)


# --------------------------------------------------------------------------- #
# Main                                                                          #
# --------------------------------------------------------------------------- #


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 2.6.4 — Debug Plot Script for SVD Localization",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Directory containing the 8 localization debug CSV/JSON files.",
    )
    parser.add_argument(
        "--map",
        type=Path,
        default=None,
        help="Path to rf_map_v1.json (for landmark overlay in Plot 5).",
    )
    parser.add_argument(
        "--frame",
        type=int,
        default=None,
        help=(
            "Frame index to visualize in the association overlay plot. "
            "Defaults to the first OK frame."
        ),
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help=(
            "Directory to save output PNG plots. "
            "If omitted, plots are shown interactively."
        ),
    )
    parser.add_argument(
        "--residual-threshold",
        type=float,
        default=0.08,
        help="Residual RMSE threshold line drawn in Plot 2 (default: 0.08 m).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir: Path = args.output

    # ---------------------------------------------------------------------- #
    # Load CSV files                                                           #
    # ---------------------------------------------------------------------- #
    try:
        poses_rows = _load_csv(out_dir / "poses.csv")
        geometry_rows = _load_csv(out_dir / "geometry_debug.csv")
        frame_rows = _load_csv(out_dir / "frame_debug.csv")
        assoc_rows = _load_csv(out_dir / "association_debug.csv")
    except FileNotFoundError as exc:
        logger.error("Missing required output file: %s", exc)
        return 1

    # ---------------------------------------------------------------------- #
    # Plot 1 — Trajectory                                                      #
    # ---------------------------------------------------------------------- #
    frame_indices, xs, ys, yaws, statuses, is_fallbacks = _extract_poses(poses_rows)
    if not xs:
        logger.warning("No valid pose rows found — skipping trajectory plot.")
    else:
        _plot_trajectory(frame_indices, xs, ys, yaws, is_fallbacks, args.save)

    # ---------------------------------------------------------------------- #
    # Plot 2 — Residuals                                                       #
    # ---------------------------------------------------------------------- #
    res_fidxs, residuals = _extract_residuals(poses_rows)
    if residuals:
        _plot_residuals(res_fidxs, residuals, args.residual_threshold, args.save)
    else:
        logger.warning("No residual data found — skipping residual plot.")

    # ---------------------------------------------------------------------- #
    # Plot 3 — Geometry                                                        #
    # ---------------------------------------------------------------------- #
    geo_fidxs, spreads, conds, geo_warnings = _extract_geometry(geometry_rows)
    if geo_fidxs:
        _plot_geometry(geo_fidxs, spreads, conds, geo_warnings, args.save)

    # ---------------------------------------------------------------------- #
    # Plot 4 — Status Summary                                                  #
    # ---------------------------------------------------------------------- #
    status_counts = _extract_frame_status(frame_rows)
    if status_counts:
        _plot_status_summary(status_counts, args.save)

    # ---------------------------------------------------------------------- #
    # Plot 5 — Association Overlay for one frame                               #
    # ---------------------------------------------------------------------- #
    landmarks: List[Dict[str, Any]] = []
    if args.map is not None:
        try:
            landmarks = _load_rf_map(args.map)
        except Exception as exc:
            logger.warning("Could not load RF map: %s", exc)

    # Determine which frame to visualize
    target_frame: Optional[int] = args.frame
    if target_frame is None:
        # Pick the first OK (non-fallback) frame that has association data
        ok_frames = [
            int(r["frame_index"])
            for r in poses_rows
            if r.get("status", "") == "OK" and r.get("is_fallback", "false") == "false"
        ]
        if ok_frames:
            target_frame = ok_frames[0]
        elif poses_rows:
            target_frame = int(poses_rows[0]["frame_index"])

    if target_frame is not None:
        xl, yl, xm, ym = _extract_association_for_frame(assoc_rows, target_frame)
        if xl:
            _plot_frame_association(
                target_frame, xl, yl, xm, ym, landmarks, poses_rows, args.save
            )
        else:
            logger.warning(
                "No association data found for frame %d — skipping overlay plot.",
                target_frame,
            )
    else:
        logger.warning("No valid frame available for association overlay.")

    logger.info("All plots generated successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
