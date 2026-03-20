import os
import pandas as pd
import matplotlib.pyplot as plt
import argparse

# Global results directory (single source of truth)
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--wheel", default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv"))
    ap.add_argument("--ekf", default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv"))
    ap.add_argument("--orb", default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv"))
    ap.add_argument("--out", default=os.path.join(RESULTS_DIR, "trajectory_overlay.png"))
    args = ap.parse_args()

    # Load CSVs
    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)
    orb = pd.read_csv(args.orb)

    plt.figure(figsize=(8,8))

    # GT from wheel file (same GT for all estimators)
    plt.plot(wheel["gt_x"], wheel["gt_y"], label="GT", linewidth=2)

    # Estimators
    plt.plot(wheel["est_x_al"], wheel["est_y_al"], label="Wheel", alpha=0.8)
    plt.plot(ekf["est_x_al"], ekf["est_y_al"], label="EKF", alpha=0.8)
    plt.plot(orb["est_x_al"], orb["est_y_al"], label="ORB", alpha=0.8)

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Trajectory Overlay (Aligned)")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    # Ensure output directory exists
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=300)
    print(f"Saved: {args.out}")

    plt.show()
