#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ========================= GLOBAL RESULTS DIR =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")
# =====================================================================

def wrap_pi(a):
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

def cross_along_error(gt_x, gt_y, gt_yaw, est_x, est_y):
    """
    Compute cross-track (CTE) and along-track (ALE) errors.
    gt_x, gt_y: ground truth positions
    gt_yaw: ground truth heading
    est_x, est_y: estimated positions
    Returns: cte, ale (arrays)
    """
    ex = gt_x - est_x
    ey = gt_y - est_y
    # Tangent (along-path) and normal (perpendicular) unit vectors
    t_x = np.cos(gt_yaw)
    t_y = np.sin(gt_yaw)
    n_x = -t_y
    n_y = t_x
    ale = ex * t_x + ey * t_y
    cte = ex * n_x + ey * n_y
    return cte, ale

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Compute Cross-Track and Along-Track Errors")
    
    # ========================= INPUTS =========================
    ap.add_argument("--wheel", default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv"))
    ap.add_argument("--ekf", default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv"))
    ap.add_argument("--orb", default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv"))  # ✅ ADDED
    ap.add_argument("--out_csv", default=os.path.join(RESULTS_DIR, "cross_along_error.csv"))
    ap.add_argument("--out_plot", default=os.path.join(RESULTS_DIR, "cross_along_error_plot.png"))
    
    args = ap.parse_args()

    # ========================= LOAD DATA =========================
    wheel = pd.read_csv(args.wheel)
    ekf   = pd.read_csv(args.ekf)
    orb   = pd.read_csv(args.orb)  # ✅ ADDED

    # -------------------------------
    # Common time horizon
    # -------------------------------
    t_end = min(
        float(wheel["t_s"].max()),
        float(ekf["t_s"].max()),
        float(orb["t_s"].max())
    )
    
    wheel = wheel[wheel["t_s"] <= t_end].reset_index(drop=True)
    ekf   = ekf[ekf["t_s"] <= t_end].reset_index(drop=True)
    orb   = orb[orb["t_s"] <= t_end].reset_index(drop=True)

    # ========================= COMPUTE =========================
    # Compute CTE and ALE for all estimators
    wheel_cte, wheel_ale = cross_along_error(
        wheel["gt_x"].values, wheel["gt_y"].values, wheel["gt_yaw"].values,
        wheel["est_x_al"].values, wheel["est_y_al"].values
    )

    ekf_cte, ekf_ale = cross_along_error(
        ekf["gt_x"].values, ekf["gt_y"].values, ekf["gt_yaw"].values,
        ekf["est_x_al"].values, ekf["est_y_al"].values
    )
    
    orb_cte, orb_ale = cross_along_error(  # ✅ ADDED
        orb["gt_x"].values, orb["gt_y"].values, orb["gt_yaw"].values,
        orb["est_x_al"].values, orb["est_y_al"].values
    )

    # ========================= SAVE CSV =========================
    df_out = pd.DataFrame({
        "t_s": wheel["t_s"],
        "wheel_cte": wheel_cte,
        "wheel_ale": wheel_ale,
        "ekf_cte": ekf_cte,
        "ekf_ale": ekf_ale,
        "orb_cte": orb_cte,  # ✅ ADDED
        "orb_ale": orb_ale   # ✅ ADDED
    })
    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    df_out.to_csv(args.out_csv, index=False)
    print(f"Saved CSV: {args.out_csv}")

    # ========================= PLOT =========================
    plt.figure(figsize=(12,5))
    
    # Plot CTE and ALE for each estimator
    plt.plot(wheel["t_s"], wheel_cte, label="Wheel Cross-Track", color="r", alpha=0.8)
    plt.plot(ekf["t_s"], ekf_cte, label="EKF Cross-Track", color="r", linestyle="--", alpha=0.8)
    plt.plot(orb["t_s"], orb_cte, label="ORB Cross-Track", color="r", linestyle=":", alpha=0.8)  # ✅ ADDED
    
    plt.plot(wheel["t_s"], wheel_ale, label="Wheel Along-Track", color="b", alpha=0.8)
    plt.plot(ekf["t_s"], ekf_ale, label="EKF Along-Track", color="b", linestyle="--", alpha=0.8)
    plt.plot(orb["t_s"], orb_ale, label="ORB Along-Track", color="b", linestyle=":", alpha=0.8)  # ✅ ADDED

    # Labels and plot formatting
    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    plt.title("Cross-Track and Along-Track Error vs Time")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    
    # Save plot
    plt.savefig(args.out_plot, dpi=300)
    print(f"Saved plot: {args.out_plot}")
    plt.show()
