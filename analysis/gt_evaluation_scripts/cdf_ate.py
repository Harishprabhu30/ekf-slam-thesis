#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def ate_series(gt_x, gt_y, est_x, est_y):
    """Compute point-wise Absolute Trajectory Error (ATE)."""
    ex = gt_x - est_x
    ey = gt_y - est_y
    return np.sqrt(ex**2 + ey**2)

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="CDF of Absolute Trajectory Error")
    ap.add_argument("--wheel", default="analysis/results_gt_traj_v3/wheel_synced_aligned.csv")
    ap.add_argument("--ekf", default="analysis/results_gt_traj_v3/ekf_synced_aligned.csv")
    ap.add_argument("--out_csv", default="analysis/results_gt_traj_v3/ate_cdf.csv")
    ap.add_argument("--out_plot", default="analysis/results_gt_traj_v3/ate_cdf_plot.png")
    args = ap.parse_args()

    # Load CSV
    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)

    # Align time horizon
    t_end = min(wheel["t_s"].max(), ekf["t_s"].max())
    wheel = wheel[wheel["t_s"] <= t_end].reset_index(drop=True)
    ekf = ekf[ekf["t_s"] <= t_end].reset_index(drop=True)

    # Compute ATE
    wheel_ate = ate_series(wheel["gt_x"], wheel["gt_y"], wheel["est_x_al"], wheel["est_y_al"])
    ekf_ate   = ate_series(ekf["gt_x"], ekf["gt_y"], ekf["est_x_al"], ekf["est_y_al"])

    # Save CSV for reference
    df_out = pd.DataFrame({
        "t_s": wheel["t_s"],
        "wheel_ate": wheel_ate,
        "ekf_ate": ekf_ate
    })
    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    df_out.to_csv(args.out_csv, index=False)
    print(f"Saved CSV: {args.out_csv}")

    # Compute CDFs
    wheel_sorted = np.sort(wheel_ate)
    ekf_sorted   = np.sort(ekf_ate)
    wheel_cdf = np.arange(1, len(wheel_sorted)+1) / len(wheel_sorted)
    ekf_cdf   = np.arange(1, len(ekf_sorted)+1) / len(ekf_sorted)

    # Plot CDF
    plt.figure(figsize=(8,5))
    plt.plot(wheel_sorted, wheel_cdf, label="Wheel", linewidth=2)
    plt.plot(ekf_sorted, ekf_cdf, label="EKF", linewidth=2, linestyle="--")
    plt.xlabel("ATE [m]")
    plt.ylabel("Cumulative Probability")
    plt.title("CDF of Absolute Trajectory Error")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out_plot, dpi=300)
    print(f"Saved plot: {args.out_plot}")
    plt.show()
