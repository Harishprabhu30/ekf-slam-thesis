from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

ABL_ROOT = ROOT / "ablations" / "default_vs_tuned_t01"
TABLES = ABL_ROOT / "tables"
PLOTS = ABL_ROOT / "plots"

PLOTS.mkdir(parents=True, exist_ok=True)

COMPARISON_CSV = TABLES / "orbslam3_default_vs_tuned_t01.csv"
EFFECT_CSV = TABLES / "orbslam3_ablation_effectiveness_t01.csv"

LIGHTINGS = ["bright", "dim", "lowlight"]
CONFIGS = ["default_1000_20_7", "tuned_1800_10_4"]

LABELS = {
    "default_1000_20_7": "Default\n1000/20/7",
    "tuned_1800_10_4": "Tuned\n1800/10/4",
}

COLORS = {
    "default_1000_20_7": "tab:blue",
    "tuned_1800_10_4": "tab:red",
}

PLOT_SPECS = [
    ("ate_rmse_m", "ATE RMSE [m]", "ablation_ate_rmse_default_vs_tuned.png"),
    ("ate_norm_rmse", "Normalized ATE RMSE [m/m]", "ablation_normalized_ate_default_vs_tuned.png"),
    ("yaw_abs_mean_deg", "Mean Absolute Yaw Error [deg]", "ablation_yaw_default_vs_tuned.png"),
    ("rpe_trans_rmse_m_1.0s", "RPE Translation RMSE @1s [m]", "ablation_rpe_default_vs_tuned.png"),
    ("pose_count_ratio", "Pose Output Ratio", "ablation_pose_output_ratio_default_vs_tuned.png"),
]

def grouped_bar(df, metric, ylabel, filename):
    x = np.arange(len(LIGHTINGS))
    width = 0.34

    plt.figure(figsize=(9, 5))

    for i, cfg in enumerate(CONFIGS):
        vals = []

        for lighting in LIGHTINGS:
            row = df[(df["lighting"] == lighting) & (df["config"] == cfg)]
            vals.append(float(row[metric].iloc[0]) if not row.empty else np.nan)

        offset = (i - 0.5) * width

        plt.bar(
            x + offset,
            vals,
            width,
            label=LABELS[cfg],
            color=COLORS[cfg],
            alpha=0.9,
        )

        for j, v in enumerate(vals):
            if not np.isnan(v):
                plt.text(
                    x[j] + offset,
                    v,
                    f"{v:.2f}",
                    ha="center",
                    va="bottom",
                    fontsize=8,
                )

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.xlabel("Lighting condition")
    plt.ylabel(ylabel)
    plt.title(f"ORB-SLAM3 Ablation — {ylabel}")
    plt.grid(axis="y", alpha=0.35)
    plt.legend()
    plt.tight_layout()

    out = PLOTS / filename
    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def improvement_plot(effect):
    metrics = [
        ("tuned_ate_improvement_pct", "ATE RMSE improvement [%]"),
        ("tuned_norm_ate_improvement_pct", "Normalized ATE improvement [%]"),
        ("tuned_yaw_improvement_pct", "Yaw error improvement [%]"),
        ("tuned_rpe_improvement_pct", "RPE improvement [%]"),
    ]

    x = np.arange(len(LIGHTINGS))
    width = 0.2

    plt.figure(figsize=(11, 5.2))

    for i, (col, label) in enumerate(metrics):
        vals = []

        for lighting in LIGHTINGS:
            row = effect[effect["lighting"] == lighting]
            vals.append(float(row[col].iloc[0]) if not row.empty else np.nan)

        offset = (i - 1.5) * width

        plt.bar(
            x + offset,
            vals,
            width,
            label=label,
            alpha=0.9,
        )

    plt.axhline(0.0, color="black", linewidth=1.0)
    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.ylabel("Improvement of tuned over default [%]")
    plt.xlabel("Lighting condition")
    plt.title("ORB-SLAM3 Parameter Ablation Effectiveness\nPositive = tuned configuration improves over default")
    plt.grid(axis="y", alpha=0.35)
    plt.legend(fontsize=8)
    plt.tight_layout()

    out = PLOTS / "ablation_tuned_improvement_percent.png"
    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def main():
    if not COMPARISON_CSV.exists():
        raise FileNotFoundError(COMPARISON_CSV)

    df = pd.read_csv(COMPARISON_CSV)

    for metric, ylabel, filename in PLOT_SPECS:
        grouped_bar(df, metric, ylabel, filename)

    if EFFECT_CSV.exists():
        effect = pd.read_csv(EFFECT_CSV)
        improvement_plot(effect)

    print(f"\nAblation plots saved in: {PLOTS}")

if __name__ == "__main__":
    main()
