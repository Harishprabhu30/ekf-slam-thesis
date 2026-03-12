import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse


def compute_distance(df):
    x = df["gt_x"].values
    y = df["gt_y"].values

    dist = np.zeros(len(x))

    for i in range(1, len(x)):
        dx = x[i] - x[i-1]
        dy = y[i] - y[i-1]
        dist[i] = dist[i-1] + np.sqrt(dx*dx + dy*dy)

    return dist


def compute_ate(df):
    ex = df["gt_x"] - df["est_x_al"]
    ey = df["gt_y"] - df["est_y_al"]

    return np.sqrt(ex*ex + ey*ey)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--wheel", default="analysis/results_gt_traj_v3/wheel_synced_aligned.csv")
    parser.add_argument("--ekf", default="analysis/results_gt_traj_v3/ekf_synced_aligned.csv")

    args = parser.parse_args()

    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)

    dist = compute_distance(wheel)

    ate_wheel = compute_ate(wheel)
    ate_ekf = compute_ate(ekf)

    plt.figure(figsize=(8,5))

    plt.plot(dist, ate_wheel, label="Wheel", linewidth=2)
    plt.plot(dist, ate_ekf, label="EKF", linewidth=2)

    plt.xlabel("Distance traveled (m)")
    plt.ylabel("Absolute Trajectory Error (m)")
    plt.title("ATE vs Distance Traveled")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()

    out="analysis/results_gt_traj_v3/ate_vs_distance.png"
    plt.savefig(out)

    print("Saved:", out)
