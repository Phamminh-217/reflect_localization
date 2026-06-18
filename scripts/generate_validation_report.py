#!/usr/bin/env python3
"""Script to generate a comprehensive field validation report from localization results."""

import argparse
import logging
import subprocess
import sys
from pathlib import Path
from typing import Optional

# Setup path so we can import from src/
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from rf_threshold.localization.pose_evaluator import evaluate_poses_from_results

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("generate_validation_report")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a comprehensive localization validation report."
    )
    parser.add_argument(
        "-d",
        "--results-dir",
        type=str,
        required=True,
        help="Path to the directory containing poses.csv and other localization debug output files.",
    )
    parser.add_argument(
        "-m",
        "--map",
        type=str,
        default=None,
        help="Path to the global RF map JSON file (for plotting landmark overlay).",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Path to save the validation_report.md. Defaults to <results-dir>/validation_report.md",
    )
    return parser.parse_args()


def run_plot_script(results_dir: Path, map_path: Optional[Path]) -> bool:
    """Run plot_localization_debug.py via subprocess to generate PNG plots."""
    logger.info("Generating debug plots using plot_localization_debug.py...")
    cmd = [
        sys.executable,
        str(Path(__file__).parent / "plot_localization_debug.py"),
        "--output",
        str(results_dir),
        "--save",
        str(results_dir),
    ]
    if map_path:
        cmd.extend(["--map", str(map_path)])

    try:
        res = subprocess.run(cmd, check=True, capture_output=True, text=True)
        logger.info("Successfully generated debug plots: %s", res.stdout.strip())
        return True
    except subprocess.CalledProcessError as exc:
        logger.error("Failed to run plot script: %s\nStdout: %s\nStderr: %s", exc, exc.stdout, exc.stderr)
        return False


def generate_report(results_dir: Path, map_path: Optional[Path], output_path: Path) -> int:
    """Generate the markdown validation report."""
    if not results_dir.exists():
        logger.error("Results directory does not exist: %s", results_dir)
        return 1

    poses_csv = results_dir / "poses.csv"
    if not poses_csv.exists():
        logger.error("poses.csv not found in results directory: %s", poses_csv)
        return 1

    # 1. Run plot generator
    plots_ok = run_plot_script(results_dir, map_path)
    if not plots_ok:
        logger.warning("Continuing report generation without guaranteeing up-to-date plots.")

    # 2. Evaluate pose metrics
    logger.info("Evaluating pose metrics from %s...", poses_csv)
    try:
        metrics = evaluate_poses_from_results(results_dir)
    except Exception as exc:
        logger.error("Failed to evaluate pose metrics: %s", exc)
        return 1

    # 3. Construct Markdown report contents
    logger.info("Writing validation report to %s...", output_path)

    report_lines = []
    report_lines.append("# Báo Cáo Thực Nghiệm Định Vị RF-SVD (Field Validation Report)")
    report_lines.append("")
    report_lines.append(f"- **Thư mục kết quả:** `{results_dir}`")
    if map_path:
        report_lines.append(f"- **RF Map:** `{map_path}`")
    report_lines.append("")
    
    report_lines.append("## 1. Thống Kê Tổng Quan (Aggregate Metrics)")
    report_lines.append("")
    report_lines.append("| Chỉ số (Metric) | Giá trị (Value) | Tỷ lệ (Ratio) |")
    report_lines.append("|---|---|---|")
    report_lines.append(f"| **Tổng số frame** (Total Frames) | {metrics.num_frames} | 100.0% |")
    report_lines.append(f"| **Frame SVD OK** (SVD Success) | {metrics.num_ok} | {metrics.ok_rate:.1%} |")
    report_lines.append(f"| **Frame Fallback** (Fallback Mode) | {metrics.num_fallback} | {metrics.fallback_rate:.1%} |")
    report_lines.append(f"| **Frame Bị Reject** (Rejected) | {metrics.num_rejected} | {metrics.rejection_rate:.1%} |")
    
    mean_rmse_str = f"{metrics.mean_residual_rmse:.4f} m" if metrics.mean_residual_rmse is not None else "N/A"
    max_rmse_str = f"{metrics.max_residual_rmse:.4f} m" if metrics.max_residual_rmse is not None else "N/A"
    
    report_lines.append(f"| **SVD Residual RMSE Trung bình** | {mean_rmse_str} | - |")
    report_lines.append(f"| **SVD Residual RMSE Lớn nhất** | {max_rmse_str} | - |")
    report_lines.append(f"| **Chuỗi Fallback liên tiếp tối đa** | {metrics.max_consecutive_fallback} frames | - |")
    report_lines.append("")

    report_lines.append("## 2. Biểu Đồ & Trực Quan Hóa (Plots & Visualization)")
    report_lines.append("")
    
    # Trajectory Plot
    if (results_dir / "01_trajectory.png").exists():
        report_lines.append("### Quỹ đạo Robot (Estimated Trajectory)")
        report_lines.append("![Robot Trajectory](01_trajectory.png)")
        report_lines.append("")

    # Status Summary
    if (results_dir / "04_status_summary.png").exists():
        report_lines.append("### Phân bố Trạng thái Định vị (Localization Status Summary)")
        report_lines.append("![Status Summary](04_status_summary.png)")
        report_lines.append("")

    # Residuals
    if (results_dir / "02_residuals.png").exists():
        report_lines.append("### SVD Residual RMSE theo Frame (SVD Residuals)")
        report_lines.append("![SVD Residuals](02_residuals.png)")
        report_lines.append("")

    # Geometry Spread
    if (results_dir / "03_geometry.png").exists():
        report_lines.append("### Phân bố Hình học Landmark & Số lượng Match (Geometry spread)")
        report_lines.append("![Geometry Analysis](03_geometry.png)")
        report_lines.append("")

    # 4. Warnings & Diagnostics
    report_lines.append("## 3. Cảnh Báo Hệ Thống (System Warnings)")
    report_lines.append("")
    if not metrics.warnings:
        report_lines.append("> [!NOTE]")
        report_lines.append("> Không phát hiện cảnh báo hoặc dấu hiệu bất thường nào. Hệ thống hoạt động tốt! ✅")
    else:
        for warn in metrics.warnings:
            if "CRITICAL" in warn or "Zero OK" in warn:
                report_lines.append(f"> [!CAUTION]")
                report_lines.append(f"> {warn}")
            elif "Drift warning" in warn or "High fallback ratio" in warn or "HIGH_RESIDUAL" in warn:
                report_lines.append(f"> [!WARNING]")
                report_lines.append(f"> {warn}")
            else:
                report_lines.append(f"> [!NOTE]")
                report_lines.append(f"> {warn}")
            report_lines.append("")

    # 5. Diagnostic Checklist Recommendations
    report_lines.append("## 4. Khuyến Nghị Gỡ Lỗi (Debugging Recommendations)")
    report_lines.append("")
    
    if metrics.ok_rate < 0.50:
        report_lines.append("- **[!] Tỷ lệ định vị thành công thấp (< 50%):**")
        report_lines.append("  - Kiểm tra `frame_debug.csv` để xem số lượng RF detection thu được từ Phase 1.")
        report_lines.append("  - Nếu số lượng detection luôn < 3, hãy kiểm tra lại ngưỡng threshold trong Phase 1 hoặc mật độ RF landmark vật lý.")
        report_lines.append("  - Xem chi tiết tại **Step 1 & Step 2** trong [DEBUG_LOG_PHASE2.md](file:///home/minh/rf_threshold_localization/DOCS/PHASE2/DEBUG_LOG_PHASE2.md).")
        
    if metrics.mean_residual_rmse is not None and metrics.mean_residual_rmse > 0.08:
        report_lines.append("- **[!] Sai số SVD residual lớn (> 0.08m):**")
        report_lines.append("  - Giá trị này ám chỉ khoảng cách giữa các detection và landmark thật sau khi align bị lệch nhiều.")
        report_lines.append("  - Có khả năng cao tọa độ landmark trong RF map bị đo đạc sai, hoặc cấu hình extrinsic calibration bị lệch.")
        report_lines.append("  - Đọc hướng dẫn chẩn đoán lỗi tại **Step 3 & Step 5** trong [DEBUG_LOG_PHASE2.md](file:///home/minh/rf_threshold_localization/DOCS/PHASE2/DEBUG_LOG_PHASE2.md).")
        
    if metrics.drift_warning:
        report_lines.append("- **[!] Cảnh báo Drift xảy ra do chuỗi Fallback quá dài:**")
        report_lines.append("  - Robot di chuyển qua vùng không đủ 3 landmark khớp nhau trong nhiều frame liên tiếp.")
        report_lines.append("  - Nếu robot di chuyển nhanh, sai số tích lũy sẽ rất lớn. Khuyến nghị tăng mật độ landmark ở các khu vực này.")
        report_lines.append("  - Xem chi tiết tại **Step 6** trong [DEBUG_LOG_PHASE2.md](file:///home/minh/rf_threshold_localization/DOCS/PHASE2/DEBUG_LOG_PHASE2.md).")

    if not (metrics.ok_rate < 0.50 or (metrics.mean_residual_rmse is not None and metrics.mean_residual_rmse > 0.08) or metrics.drift_warning):
        report_lines.append("- **Hệ thống hoạt động ổn định:** Không cần hành động khẩn cấp.")
        report_lines.append("- Nên kiểm tra ngẫu nhiên một vài frame (ví dụ frame có residual cao nhất) bằng cách chạy `plot_localization_debug.py` chỉ định frame cụ thể.")

    report_lines.append("")

    try:
        with output_path.open("w", encoding="utf-8") as f:
            f.write("\n".join(report_lines))
        logger.info("Successfully generated validation report.")
        return 0
    except Exception as exc:
        logger.error("Failed to write validation report: %s", exc)
        return 1


def main() -> int:
    args = parse_args()
    results_dir = Path(args.results_dir)
    map_path = Path(args.map) if args.map else None
    
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = results_dir / "validation_report.md"

    return generate_report(results_dir, map_path, output_path)


if __name__ == "__main__":
    sys.exit(main())
