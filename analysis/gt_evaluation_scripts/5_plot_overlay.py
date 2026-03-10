import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--wheel", required=True)
    ap.add_argument("--ekf", required=True)
    ap.add_argument("--out", default="analysis/results_gt_traj_v3/trajectory_overlay.png")
    args = ap.parse_args()

    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)

    plt.figure(figsize=(8,8))

    # GT from wheel file (same GT for both)
    plt.plot(wheel["gt_x"], wheel["gt_y"], label="GT", linewidth=2)

    # Wheel alignedA
    plt.plot(wheel["est_x_al"], wheel["est_y_al"], label="Wheel", alpha=0.8)

    # EKF aligned
    plt.plot(ekf["est_x_al"], ekf["est_y_al"], label="EKF", alpha=0.8)

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Trajectory Overlay (Aligned)")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=300)
    print(f"Saved: {args.out}")

    plt.show()
