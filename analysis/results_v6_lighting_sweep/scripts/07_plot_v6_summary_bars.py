from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

SUMMARY = ROOT / "tables" / "v6_mean_std_by_lighting_estimator.csv"
PLOTS = ROOT / "plots" / "summary_bars"

PLOTS.mkdir(parents=True, exist_ok=True)

LIGHTINGS = ["bright", "dim", "lowlight"]
ESTIMATORS = ["wheel", "ekf", "orbslam3"]

EST_LABEL = {
    "wheel": "Wheel",
    "ekf": "EKF",
    "orbslam3": "ORB-SLAM3",
}

PLOT_SPECS = [
    {
        "metric": "ate_rmse_m",
        "ylabel": "ATE RMSE [m]",
        "title": "ATE RMSE Mean ± Std Across Trials",
        "filename": "summary_ate_rmse_mean_std.png",
    },
    {
        "metric": "ate_norm_rmse",
        "ylabel": "Normalized ATE RMSE [m/m]",
        "title": "Normalized ATE RMSE Mean ± Std Across Trials",
        "filename": "summary_normalized_ate_mean_std.png",
    },
    {
        "metric": "drift_rate_m_per_m",
        "ylabel": "Drift Rate [m/m]",
        "title": "Drift Rate Mean ± Std Across Trials",
        "filename": "summary_drift_rate_mean_std.png",
    },
    {
        "metric": "yaw_abs_mean_deg",
        "ylabel": "Mean Absolute Yaw Error [deg]",
        "title": "Yaw Error Mean ± Std Across Trials",
        "filename": "summary_yaw_error_mean_std.png",
    },
    {
        "metric": "rpe_trans_rmse_m_1.0s",
        "ylabel": "RPE Translation RMSE @1s [m]",
        "title": "RPE Translation Mean ± Std Across Trials",
        "filename": "summary_rpe_translation_mean_std.png",
    },
    {
        "metric": "pose_count_ratio",
        "ylabel": "Pose Output Ratio",
        "title": "Pose Output Ratio Mean ± Std Across Trials",
        "filename": "summary_pose_output_ratio_mean_std.png",
    },
]

def plot_metric(summary, metric, ylabel, title, filename):
    x = np.arange(len(LIGHTINGS))
    width = 0.25

    plt.figure(figsize=(10, 5))

    for i, est in enumerate(ESTIMATORS):
        means = []
        stds = []

        for lighting in LIGHTINGS:
            row = summary[
                (summary["lighting"] == lighting) &
                (summary["estimator"] == est)
            ]

            if row.empty:
                means.append(np.nan)
                stds.append(0.0)
            else:
                means.append(float(row[f"{metric}_mean"].iloc[0]))
                stds.append(float(row[f"{metric}_std"].iloc[0]))

        offset = (i - 1) * width

        plt.bar(
            x + offset,
            means,
            width,
            yerr=stds,
            capsize=5,
            label=EST_LABEL.get(est, est),
        )

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.ylabel(ylabel)
    plt.xlabel("Lighting condition")
    plt.title(title)
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()

    out = PLOTS / filename
    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def plot_orb_only(summary):
    orb = summary[summary["estimator"] == "orbslam3"].copy()

    metrics = [
        ("ate_rmse_m", "ATE RMSE [m]", "orb_only_ate_rmse.png"),
        ("ate_norm_rmse", "Normalized ATE RMSE [m/m]", "orb_only_normalized_ate.png"),
        ("pose_count_ratio", "Pose Output Ratio", "orb_only_pose_output_ratio.png"),
    ]

    for metric, ylabel, filename in metrics:
        means = []
        stds = []

        for lighting in LIGHTINGS:
            row = orb[orb["lighting"] == lighting]

            if row.empty:
                means.append(np.nan)
                stds.append(0.0)
            else:
                means.append(float(row[f"{metric}_mean"].iloc[0]))
                stds.append(float(row[f"{metric}_std"].iloc[0]))

        x = np.arange(len(LIGHTINGS))

        plt.figure(figsize=(7, 5))
        plt.bar(x, means, yerr=stds, capsize=5)
        plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
        plt.ylabel(ylabel)
        plt.xlabel("Lighting condition")
        plt.title(f"ORB-SLAM3 {ylabel} Mean ± Std")
        plt.grid(axis="y", linestyle="--", alpha=0.6)
        plt.tight_layout()

        out = PLOTS / filename
        plt.savefig(out, dpi=300)
        plt.close()

        print(f"Saved: {out}")

def main():
    if not SUMMARY.exists():
        raise FileNotFoundError(f"Summary table not found: {SUMMARY}")

    summary = pd.read_csv(SUMMARY)

    for spec in PLOT_SPECS:
        metric = spec["metric"]
        mean_col = f"{metric}_mean"
        std_col = f"{metric}_std"

        if mean_col not in summary.columns or std_col not in summary.columns:
            print(f"Skipping {metric}: columns missing")
            continue

        plot_metric(
            summary,
            metric=metric,
            ylabel=spec["ylabel"],
            title=spec["title"],
            filename=spec["filename"],
        )

    plot_orb_only(summary)

    print("\nDone summary plots.")
    print(f"Output folder: {PLOTS}")

if __name__ == "__main__":
    main()
