#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ========================= GLOBAL RESULTS DIR =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")
# =====================================================================

EPS_DS = 1e-2  # minimum delta_s to compute curvature

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def compute_curvature(gt_x, gt_y, gt_yaw):
    dx = np.diff(gt_x)
    dy = np.diff(gt_y)
    ds = np.sqrt(dx**2 + dy**2)
    dtheta = np.diff(gt_yaw)
    dtheta = np.array([wrap_pi(a) for a in dtheta])
    curvature = np.zeros_like(gt_x)
    curvature[1:] = np.abs(dtheta) / np.maximum(ds, EPS_DS)
    curvature[0] = curvature[1]
    return curvature, ds

def ate_series(gt_x, gt_y, est_x, est_y):
    ex = gt_x - est_x
    ey = gt_y - est_y
    return np.sqrt(ex*ex + ey*ey)

def yaw_error_series(gt_yaw, est_yaw):
    return np.array([wrap_pi(a-b) for a,b in zip(est_yaw, gt_yaw)])

def curvature_bins(curv, value, bins=[0,0.05,0.2,np.inf], labels=["Low","Medium","High"]):
    bin_ids = np.digitize(curv, bins) - 1
    df = pd.DataFrame({"curvature_bin": [labels[i] for i in bin_ids], "value": value})
    return df.groupby("curvature_bin")["value"].mean().reset_index()

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()

    # ========================= INPUTS =========================
    ap.add_argument("--wheel", default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv"))
    ap.add_argument("--ekf", default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv"))
    ap.add_argument("--orb", default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv"))  # ✅ ADDED
    ap.add_argument("--out_csv", default=os.path.join(RESULTS_DIR, "curvature_error_yaw_analysis.csv"))
    ap.add_argument("--out_scatter", default=os.path.join(RESULTS_DIR, "curvature_vs_ate_scatter.png"))
    ap.add_argument("--out_time", default=os.path.join(RESULTS_DIR, "curvature_vs_time.png"))
    ap.add_argument("--out_bins", default=os.path.join(RESULTS_DIR, "curvature_error_bins.png"))
    ap.add_argument("--out_yaw", default=os.path.join(RESULTS_DIR, "yaw_error_vs_curvature.png"))
    
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
    curvature, ds = compute_curvature(wheel["gt_x"].values, wheel["gt_y"].values, wheel["gt_yaw"].values)

    # Compute ATE and Yaw error for all estimators
    wheel_ate = ate_series(wheel["gt_x"], wheel["gt_y"], wheel["est_x_al"], wheel["est_y_al"])
    ekf_ate   = ate_series(ekf["gt_x"], ekf["gt_y"], ekf["est_x_al"], ekf["est_y_al"])
    orb_ate   = ate_series(orb["gt_x"], orb["gt_y"], orb["est_x_al"], orb["est_y_al"])  # ✅ ADDED
    
    wheel_yaw_err = yaw_error_series(wheel["gt_yaw"], wheel["est_yaw_al"])
    ekf_yaw_err   = yaw_error_series(ekf["gt_yaw"], ekf["est_yaw_al"])
    orb_yaw_err   = yaw_error_series(orb["gt_yaw"], orb["est_yaw_al"])  # ✅ ADDED

    # ========================= SAVE CSV =========================
    df_out = pd.DataFrame({
        "t_s": wheel["t_s"],
        "curvature": curvature,
        "wheel_ate": wheel_ate,
        "ekf_ate": ekf_ate,
        "orb_ate": orb_ate,  # ✅ ADDED
        "wheel_yaw_err": wheel_yaw_err,
        "ekf_yaw_err": ekf_yaw_err,
        "orb_yaw_err": orb_yaw_err  # ✅ ADDED
    })
    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    df_out.to_csv(args.out_csv, index=False)
    print(f"Saved CSV: {args.out_csv}")

    # --------- Plot A: Curvature vs ATE scatter ---------
    plt.figure(figsize=(8,5))
    plt.scatter(curvature, wheel_ate, s=10, alpha=0.6, label="Wheel")
    plt.scatter(curvature, ekf_ate, s=10, alpha=0.6, label="EKF")
    plt.scatter(curvature, orb_ate, s=10, alpha=0.6, label="ORB")  # ✅ ADDED
    plt.xlabel("Curvature [rad/m]")
    plt.ylabel("ATE [m]")
    plt.title("Curvature vs ATE")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out_scatter, dpi=300)
    print(f"Saved plot: {args.out_scatter}")

    # --------- Plot B: Curvature & ATE vs Time (two subplots) ---------
    plt.figure(figsize=(12,6))

    # Top subplot: Curvature vs time
    plt.subplot(2,1,1)
    plt.plot(wheel["t_s"], curvature, color="k", linewidth=1.2)
    plt.ylabel("Curvature [rad/m]")
    plt.title("Curvature and ATE vs Time")
    plt.grid(True, linestyle="--", alpha=0.5)

    # Bottom subplot: ATE vs time
    plt.subplot(2,1,2)
    plt.plot(wheel["t_s"], wheel_ate, label="Wheel ATE", alpha=0.8)
    plt.plot(ekf["t_s"], ekf_ate, label="EKF ATE", alpha=0.8)
    plt.plot(orb["t_s"], orb_ate, label="ORB ATE", alpha=0.8)  # ✅ ADDED
    plt.xlabel("Time [s]")
    plt.ylabel("ATE [m]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()

    plt.tight_layout()
    plt.savefig(args.out_time, dpi=300)
    print(f"Saved plot: {args.out_time}")

    # --------- Plot C: Binned curvature ---------
    df_bins_wheel = curvature_bins(curvature, wheel_ate)
    df_bins_wheel["estimator"] = "Wheel"
    df_bins_ekf   = curvature_bins(curvature, ekf_ate)
    df_bins_ekf["estimator"] = "EKF"
    df_bins_orb   = curvature_bins(curvature, orb_ate)  # ✅ ADDED
    df_bins_orb["estimator"] = "ORB"  # ✅ ADDED
    
    df_bins = pd.concat([df_bins_wheel, df_bins_ekf, df_bins_orb], ignore_index=True)  # ✅ ADDED

    x = np.arange(len(df_bins_wheel))
    width = 0.35
    plt.figure(figsize=(8,5))
    plt.bar(x - width/2, df_bins_wheel["value"], width, label="Wheel")
    plt.bar(x + width/2, df_bins_ekf["value"], width, label="EKF")
    plt.bar(x + width/2, df_bins_orb["value"], width, label="ORB")  # ✅ ADDED
    plt.xticks(x, df_bins_wheel["curvature_bin"])
    plt.ylabel("Mean ATE [m]")
    plt.xlabel("Curvature Bin")
    plt.title("Mean ATE per Curvature Bin")
    plt.grid(axis="y", linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out_bins, dpi=300)
    print(f"Saved plot: {args.out_bins}")

    # --------- Plot D: Yaw error vs curvature ---------
    plt.figure(figsize=(8,5))
    plt.scatter(curvature, np.rad2deg(wheel_yaw_err), s=10, alpha=0.6, label="Wheel")
    plt.scatter(curvature, np.rad2deg(ekf_yaw_err), s=10, alpha=0.6, label="EKF")
    plt.scatter(curvature, np.rad2deg(orb_yaw_err), s=10, alpha=0.6, label="ORB")  # ✅ ADDED
    plt.xlabel("Curvature [rad/m]")
    plt.ylabel("Yaw error [deg]")
    plt.title("Yaw Error vs Curvature")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out_yaw, dpi=300)
    print(f"Saved plot: {args.out_yaw}")

    plt.show()
