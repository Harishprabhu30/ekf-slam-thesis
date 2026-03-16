#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

def compute_distance(df):
    """
    Compute cumulative trajectory distance along the path.
    Returns cumulative distances at each timestep.
    """
    x = df["gt_x"].values
    y = df["gt_y"].values
    dx = np.diff(x)
    dy = np.diff(y)
    # segment distances
    segment_dist = np.sqrt(dx*dx + dy*dy)
    # cumulative distance: prepend 0 for first point
    cum_dist = np.insert(np.cumsum(segment_dist), 0, 0.0)
    return cum_dist

def compute_ate(df):
    """
    Compute Absolute Trajectory Error (ATE) for all points.
    """
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    ate = np.sqrt(ex*ex + ey*ey)
    return ate

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheel", default="analysis/results_gt_traj_v3/wheel_synced_aligned.csv")
    parser.add_argument("--ekf", default="analysis/results_gt_traj_v3/ekf_synced_aligned.csv")
    parser.add_argument("--out", default="analysis/results_gt_traj_v3/ate_vs_distance.png")
    args = parser.parse_args()

    # Load CSVs
    wheel_df = pd.read_csv(args.wheel)
    ekf_df = pd.read_csv(args.ekf)

    # Compute cumulative distance and ATEs
    dist_wheel = compute_distance(wheel_df)
    dist_ekf = compute_distance(ekf_df)

    ate_wheel = compute_ate(wheel_df)
    ate_ekf = compute_ate(ekf_df)

    # Use common distance vector (interpolation) to handle mismatched lengths
    max_points = max(len(dist_wheel), len(dist_ekf))
    common_dist = np.linspace(0, min(dist_wheel[-1], dist_ekf[-1]), max_points)

    ate_wheel_interp = np.interp(common_dist, dist_wheel, ate_wheel)
    ate_ekf_interp = np.interp(common_dist, dist_ekf, ate_ekf)

    # -------- Plotting --------
    plt.figure(figsize=(10,6))
    plt.plot(common_dist, ate_wheel_interp, label="Wheel", linewidth=2)
    plt.plot(common_dist, ate_ekf_interp, label="EKF", linewidth=2)
    plt.xlabel("Trajectory Distance [m]")
    plt.ylabel("ATE [m]")
    plt.title("Absolute Trajectory Error vs Distance")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Ensure output directory exists
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=300)
    print(f"[plot_ate_vs_distance] Saved plot: {args.out}")
    plt.show()
