import os
import numpy as np
import pandas as pd
import math

PHASE_NAME = {
    1: "square",
    2: "straight", # updates as per my phase metrics markers used while ros bag recordings.
    3: "cw_rotation",
    #4: "arc",
}

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compute_metrics(df):
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    ate = np.sqrt(ex * ex + ey * ey)

    yaw_err = df["gt_yaw"].values - df["est_yaw_al"].values
    yaw_err = np.array([wrap_pi(a) for a in yaw_err])
    yaw_abs = np.abs(yaw_err)

    return {
        "ATE_RMSE_m": float(np.sqrt(np.mean(ate ** 2))),
        "ATE_Mean_m": float(np.mean(ate)),
        "Yaw_Mean_rad": float(np.mean(yaw_abs)),
        "Yaw_Max_rad": float(np.max(yaw_abs)),
        "Samples": len(df)
    }


def compute_global_excluding_stop(aligned_csv):
    df = pd.read_csv(aligned_csv)

    # exclude stop (phase 0)
    phase_df = pd.read_csv("analysis/results_gt_traj_v3/phase_metrics.csv")
    valid_phases = phase_df[phase_df["phase"] != 0]

    # get valid time intervals
    intervals = valid_phases[["phase", "t0_s", "t1_s"]].drop_duplicates()

    mask = np.zeros(len(df), dtype=bool)

    for _, row in intervals.iterrows():
        t0, t1 = row["t0_s"], row["t1_s"]
        mask |= ((df["t_s"] >= t0) & (df["t_s"] < t1))

    df_valid = df[mask]

    return compute_metrics(df_valid)


if __name__ == "__main__":

    wheel_csv = "analysis/results_gt_traj_v3/wheel_synced_aligned.csv"
    ekf_csv = "analysis/results_gt_traj_v3/ekf_synced_aligned.csv"
    phase_metrics_csv = "analysis/results_gt_traj_v3/phase_metrics.csv"

    phase_df = pd.read_csv(phase_metrics_csv)

    summary_rows = []

    # ---------------- GLOBAL ----------------
    for name, csv in [("wheel", wheel_csv), ("ekf", ekf_csv)]:
        global_metrics = compute_global_excluding_stop(csv)
        summary_rows.append({
            "Estimator": name,
            "Phase": "GLOBAL (1-3)",
            **global_metrics
        })

    # ---------------- PER PHASE ----------------
    for phase_id in [1, 2, 3]: # update as per the phase markers
        phase_name = PHASE_NAME[phase_id]

        for estimator in ["wheel", "ekf"]:
            row = phase_df[
                (phase_df["phase"] == phase_id) &
                (phase_df["estimator"] == estimator)
            ].iloc[0]

            summary_rows.append({
                "Estimator": estimator,
                "Phase": phase_name,
                "ATE_RMSE_m": row["ate_rmse_m"],
                "ATE_Mean_m": row["ate_mean_m"],  
                "Yaw_Mean_rad": row["yaw_abs_mean_rad"],
                "Yaw_Max_rad": row["yaw_abs_max_rad"],
                "Samples": row["n"]
            })

    summary = pd.DataFrame(summary_rows)

    os.makedirs("analysis/results_gt_traj_v3", exist_ok=True)
    summary.to_csv("analysis/results_gt_traj_v3/final_summary_table.csv", index=False)

    print("\n========== THESIS SUMMARY TABLE ==========\n")
    print(summary.to_string(index=False))
    print("\nSaved: analysis/results_gt_traj_v3/final_summary_table.csv")
