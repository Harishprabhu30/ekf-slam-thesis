import pandas as pd
import numpy as np
import argparse
import os

def compute_distance(df):
    """Compute total trajectory length using GT positions."""
    x = df["gt_x"].values
    y = df["gt_y"].values
    dx = np.diff(x)
    dy = np.diff(y)
    return np.sum(np.sqrt(dx*dx + dy*dy))

def compute_final_ate(df):
    """Compute final absolute trajectory error (ATE)."""
    ex = df["gt_x"] - df["est_x_al"]
    ey = df["gt_y"] - df["est_y_al"]
    ate = np.sqrt(ex*ex + ey*ey)
    return ate.iloc[-1]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheel", default="analysis/results_gt_traj_v3/wheel_synced_aligned.csv")
    parser.add_argument("--ekf", default="analysis/results_gt_traj_v3/ekf_synced_aligned.csv")
    parser.add_argument("--out_csv", default="analysis/results_gt_traj_v3/final_ate_drift.csv")
    args = parser.parse_args()

    # Load aligned trajectories
    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)

    # Compute trajectory length from GT
    traj_length = compute_distance(wheel)

    # Compute final ATE for each estimator
    wheel_final = compute_final_ate(wheel)
    ekf_final = compute_final_ate(ekf)

    # Compute drift rate = final ATE / trajectory length
    wheel_drift = wheel_final / traj_length
    ekf_drift = ekf_final / traj_length

    # Print results to console
    print("\nTrajectory length: %.3f m\n" % traj_length)
    print("Estimator   Final ATE (m)   Drift Rate (m/m)")
    print("--------------------------------------------")
    print(f"Wheel       {wheel_final:.3f}          {wheel_drift:.4f}")
    print(f"EKF         {ekf_final:.3f}          {ekf_drift:.4f}")

    # Save results to CSV
    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    df_out = pd.DataFrame([
        {"Estimator": "wheel", "Final_ATE_m": wheel_final, "Drift_Rate_m_per_m": wheel_drift},
        {"Estimator": "ekf",   "Final_ATE_m": ekf_final,   "Drift_Rate_m_per_m": ekf_drift},
    ])
    df_out.to_csv(args.out_csv, index=False)
    print(f"\nSaved results to CSV: {args.out_csv}")
