#!/usr/bin/env python3
"""
Research Benchmark Evaluation and Plotting Tool:
Asynchronous Fusion of Depth Camera and 360° LiDAR Based on
Adaptive Extended Kalman Filter with Optical Depth-Error Modeling for Human Tracking
Model: 100% ID-Free Spatiotemporal Kinematics
"""

import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description="Multi-Modal Sensor Fusion Adaptive EKF Benchmark Plotter")
    parser.add_argument("--csv", default="fusion_benchmark.csv", help="Path to input telemetry CSV log file")
    parser.add_argument("--save", default="fusion_benchmark_evaluation.png", help="Filename of output figure")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"[ERROR] CSV log file not found at: {args.csv}")
        return

    df = pd.read_csv(args.csv)
    if df.empty:
        print("[ERROR] CSV log file is empty.")
        return

    # Normalize timestamp to relative elapsed seconds
    t0 = df["timestamp_sec"].iloc[0]
    time_rel = df["timestamp_sec"] - t0

    fig, axes = plt.subplots(2, 2, figsize=(16, 11))
    fig.suptitle("Adaptive Extended Kalman Filter Multi-Modal Human Tracking Evaluation (ID-Free)",
                 fontsize=14, fontweight='bold')

    # -------------------------------------------------------------
    # Plot 1: 2D Target Trajectory in Robot Coordinate Frame (base_link)
    # -------------------------------------------------------------
    ax1 = axes[0, 0]
    sources = df["sensor_source"].unique()
    color_map = {
        "FUSED_BOTH": "mediumorchid",
        "CAMERA": "forestgreen",
        "LIDAR": "deepskyblue",
        "DEAD_RECKONING": "darkorange",
        "NONE": "gray"
    }
    for src in sources:
        subset = df[df["sensor_source"] == src]
        ax1.scatter(subset["y_est"], subset["x_est"], label=f"Sensor: {src}",
                    c=color_map.get(src, "blue"), alpha=0.75, s=18)
    ax1.set_xlabel("Lateral Position Y [m] (+Left, -Right)")
    ax1.set_ylabel("Forward Distance X [m]")
    ax1.set_title("1. 2D Target Trajectory (base_link frame)")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="upper right")

    # -------------------------------------------------------------
    # Plot 2: Dynamic Covariance Adaptation (R_cam vs R_lidar)
    # Proves quadratic depth error adaptation & crowd noise tuning
    # -------------------------------------------------------------
    ax2 = axes[0, 1]
    if "r_cam" in df.columns and "r_lidar" in df.columns:
        ax2.plot(time_rel, df["r_cam"], color="crimson", linewidth=1.8, label=r"Camera Covariance $R_{cam}(t)$")
        ax2.plot(time_rel, df["r_lidar"], color="navy", linewidth=1.8, linestyle="--", label=r"LiDAR Covariance $R_{lidar}(t)$")
        ax2.set_xlabel("Elapsed Time [s]")
        ax2.set_ylabel(r"Measurement Noise Covariance $R$ [$m^2$]")
        ax2.set_title(r"2. Dynamic Covariance Adaptation ($R_{cam}$ vs $R_{lidar}$)")
        ax2.grid(True, linestyle="--", alpha=0.6)
        ax2.legend(loc="upper right")

    # -------------------------------------------------------------
    # Plot 3: Target Distance & Bearing Profile vs Time
    # -------------------------------------------------------------
    ax3 = axes[1, 0]
    dist_data = df["dist_est"] if "dist_est" in df.columns else (df["x_est"]**2 + df["y_est"]**2)**0.5
    ax3_twin = ax3.twinx()

    l1 = ax3.plot(time_rel, dist_data, color="purple", linewidth=1.8, label="Target Distance [m]")
    l2 = []
    if "bearing_deg" in df.columns:
        l2 = ax3_twin.plot(time_rel, df["bearing_deg"], color="dodgerblue", linewidth=1.5, linestyle=":", label="Bearing Angle [°]")
        ax3_twin.set_ylabel("Azimuth Bearing [°] (+Left, -Right)", color="dodgerblue")
        ax3_twin.axhline(y=0.0, color="gray", linestyle="-", alpha=0.3)

    ax3.set_xlabel("Elapsed Time [s]")
    ax3.set_ylabel("Euclidean Distance [m]", color="purple")
    ax3.set_title("3. Target Range and Azimuth Bearing Profile")
    ax3.grid(True, linestyle="--", alpha=0.6)
    all_lines = l1 + l2
    all_labels = [line.get_label() for line in all_lines]
    ax3.legend(all_lines, all_labels, loc="upper right")

    # -------------------------------------------------------------
    # Plot 4: EKF State Uncertainty vs Crowd Density
    # -------------------------------------------------------------
    ax4 = axes[1, 1]
    uncertainty = (df["cov_x"] + df["cov_y"])**0.5
    ax4_twin = ax4.twinx()

    line1 = ax4.plot(time_rel, uncertainty, color="teal", linewidth=1.8, label=r"Position Uncertainty $\sigma_{pos}$ [m]")
    line2 = ax4_twin.step(time_rel, df["crowd_density"], where="mid", color="coral", linewidth=1.5, linestyle="-.", label="Crowd Density [persons]")

    ax4.set_xlabel("Elapsed Time [s]")
    ax4.set_ylabel(r"Position Standard Deviation $\sigma$ [m]", color="teal")
    ax4_twin.set_ylabel("Bystander Count Near Target", color="coral")
    ax4.set_title("4. Filter Estimation Uncertainty vs. Bystander Crowd Density")
    ax4.grid(True, linestyle="--", alpha=0.6)

    lines4 = line1 + line2
    labels4 = [l.get_label() for l in lines4]
    ax4.legend(lines4, labels4, loc="upper right")

    plt.tight_layout()
    plt.savefig(args.save, dpi=300)
    print(f"\033[1;32m[SUCCESS] Benchmark evaluation figure successfully saved to: {args.save}\033[0m")
    plt.show()

if __name__ == "__main__":
    main()
