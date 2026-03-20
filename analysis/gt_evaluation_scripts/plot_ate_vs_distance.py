#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

# ========================= GLOBAL RESULTS DIR =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")
# =====================================================================


def compute_distance(df):
    """
    Compute cumulative trajectory distance along the path.
    Returns cumulative distances at each timestep.
    """
    x = df["gt_x"].values
    y = df["gt_y"].values
    dx = np.diff(x)
    dy = np.diff(y)

    segment_dist = np.sqrt(dx*dx + dy*dy)

    # prepend 0 for first point
    cum_dist = np.insert(np.cumsum(segment_dist), 0, 0.0)
    return cum_dist


def compute_ate(df):
    """
    Compute Absolute Trajectory Error (ATE) for all points.
    """
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    return np.sqrt(ex*ex + ey*ey)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # ========================= INPUTS =========================
    parser.add_argument(
        "--wheel",
        default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv")
    )

    parser.add_argument(
        "--ekf",
        default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv")
    )

    parser.add_argument(
        "--orb",
        default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv")  # ✅ ADDED
    )

    parser.add_argument(
        "--out",
        default=os.path.join(RESULTS_DIR, "ate_vs_distance.png")
    )

    args = parser.parse_args()

    # ========================= LOAD =========================
    wheel_df = pd.read_csv(args.wheel)
    ekf_df = pd.read_csv(args.ekf)
    orb_df = pd.read_csv(args.orb)   # ✅ ADDED

    # -------------------------------
    # Match common time horizon
    # -------------------------------
    t_end = min(
        wheel_df["t_s"].max(),
        ekf_df["t_s"].max(),
        orb_df["t_s"].max()
    )

    wheel_df = wheel_df[wheel_df["t_s"] <= t_end].reset_index(drop=True)
    ekf_df = ekf_df[ekf_df["t_s"] <= t_end].reset_index(drop=True)
    orb_df = orb_df[orb_df["t_s"] <= t_end].reset_index(drop=True)

    # ========================= COMPUTE =========================
    dist_wheel = compute_distance(wheel_df)
    dist_ekf = compute_distance(ekf_df)
    dist_orb = compute_distance(orb_df)   # ✅ ADDED

    ate_wheel = compute_ate(wheel_df)
    ate_ekf = compute_ate(ekf_df)
    ate_orb = compute_ate(orb_df)         # ✅ ADDED

    # -------------------------------
    # Common distance axis
    # -------------------------------
    max_points = max(len(dist_wheel), len(dist_ekf), len(dist_orb))

    max_dist = min(dist_wheel[-1], dist_ekf[-1], dist_orb[-1])

    common_dist = np.linspace(0, max_dist, max_points)

    # Interpolate all
    ate_wheel_interp = np.interp(common_dist, dist_wheel, ate_wheel)
    ate_ekf_interp = np.interp(common_dist, dist_ekf, ate_ekf)
    ate_orb_interp = np.interp(common_dist, dist_orb, ate_orb)

    # ========================= PLOT =========================
    plt.figure(figsize=(10,6))

    plt.plot(common_dist, ate_wheel_interp, label="Wheel", linewidth=2)
    plt.plot(common_dist, ate_ekf_interp, label="EKF", linewidth=2)
    plt.plot(common_dist, ate_orb_interp, label="ORB", linewidth=2)  # ✅ ADDED

    plt.xlabel("Trajectory Distance [m]")
    plt.ylabel("ATE [m]")
    plt.title("Absolute Trajectory Error vs Distance")

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # ========================= SAVE =========================
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=300)

    print(f"[plot_ate_vs_distance] Saved plot: {args.out}")

    plt.show()
