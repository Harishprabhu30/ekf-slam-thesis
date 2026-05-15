from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

METRICS = ROOT / "metrics" / "v6_all_trial_metrics.csv"
MAIN_TABLE = ROOT / "tables" / "v6_paper_summary_table.csv"
MAIN_ALIGNED = ROOT / "aligned"

ABL_ROOT = ROOT / "ablations" / "default_vs_tuned_t03"
ABL_TABLE = ABL_ROOT / "tables" / "orbslam3_default_vs_tuned_t03.csv"
ABL_ALIGNED = ABL_ROOT / "aligned"

OUT = ROOT / "paper_figures_t03"
FINAL = ROOT / "final_reporting_t03"

OUT.mkdir(parents=True, exist_ok=True)
FINAL.mkdir(parents=True, exist_ok=True)

LIGHTINGS = ["bright", "dim", "lowlight"]
ESTIMATORS = ["wheel", "ekf", "orbslam3"]

COLORS = {
    "gt": "black",
    "wheel": "tab:blue",
    "ekf": "tab:orange",
    "orbslam3": "tab:red",
    "default": "tab:blue",
    "tuned": "tab:red",
}

LABELS = {
    "wheel": "Wheel",
    "ekf": "EKF",
    "orbslam3": "ORB-SLAM3",
}

def savefig(name):
    png = OUT / f"{name}.png"
    pdf = OUT / f"{name}.pdf"
    plt.savefig(png, dpi=300, bbox_inches="tight")
    plt.savefig(pdf, bbox_inches="tight")
    plt.close()
    print(f"Saved: {png}")
    print(f"Saved: {pdf}")

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def ate_series(df):
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    return np.sqrt(ex * ex + ey * ey)

def load_main_aligned(lighting, trial, estimator):
    p = MAIN_ALIGNED / f"{lighting}_{trial}_{estimator}_aligned.csv"
    if not p.exists():
        raise FileNotFoundError(p)
    return pd.read_csv(p)

def plot_main_normalized_ate():
    df = pd.read_csv(METRICS)

    x = np.arange(len(LIGHTINGS))
    width = 0.24

    plt.figure(figsize=(7.2, 4.2))

    for i, est in enumerate(ESTIMATORS):
        offset = (i - 1) * width
        means, stds = [], []

        for j, lighting in enumerate(LIGHTINGS):
            g = df[(df["lighting"] == lighting) & (df["estimator"] == est)]
            vals = pd.to_numeric(g["ate_norm_rmse"], errors="coerce").dropna().values

            means.append(float(np.mean(vals)))
            stds.append(float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0)

            jitter = np.linspace(-0.035, 0.035, len(vals))
            plt.scatter(
                np.full(len(vals), x[j] + offset) + jitter,
                vals,
                color=COLORS[est],
                edgecolor="black",
                linewidth=0.35,
                s=32,
                alpha=0.9,
                zorder=3,
            )

        plt.errorbar(
            x + offset,
            means,
            yerr=stds,
            fmt="o",
            color=COLORS[est],
            capsize=4,
            markersize=6,
            linewidth=1.5,
            label=LABELS[est],
            zorder=4,
        )

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.xlabel("Lighting condition")
    plt.ylabel("Normalized ATE RMSE [m/m]")
    plt.title("Normalized trajectory error across repeated trials")
    plt.grid(axis="y", alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig1_main_normalized_ate_trials_mean_std")

def plot_lowlight_t03_main_overlay():
    lighting = "lowlight"
    trial = "t03"

    dfs = {est: load_main_aligned(lighting, trial, est) for est in ESTIMATORS}

    plt.figure(figsize=(5.6, 5.2))

    gt = dfs["wheel"]
    plt.plot(gt["gt_x"], gt["gt_y"], color=COLORS["gt"], linewidth=2.2, label="GT")

    for est in ESTIMATORS:
        df = dfs[est]
        plt.plot(
            df["est_x_al"],
            df["est_y_al"],
            color=COLORS[est],
            linewidth=1.6,
            alpha=0.92,
            label=LABELS[est],
        )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("SE(2)-aligned trajectory, lowlight t03")
    plt.axis("equal")
    plt.grid(alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig2_lowlight_t03_trajectory_overlay_se2_aligned")

def plot_ablation_t03_bars():
    df = pd.read_csv(ABL_TABLE)

    configs = ["default_1000_20_7", "tuned_1800_10_4"]
    labels = {
        "default_1000_20_7": "Default\n1000/20/7",
        "tuned_1800_10_4": "Tuned\n1800/10/4",
    }

    x = np.arange(len(LIGHTINGS))
    width = 0.34

    plt.figure(figsize=(7.0, 4.2))

    for i, cfg in enumerate(configs):
        vals = []
        for lighting in LIGHTINGS:
            row = df[(df["lighting"] == lighting) & (df["config"] == cfg)]
            vals.append(float(row["ate_rmse_m"].iloc[0]))

        offset = (i - 0.5) * width
        color = COLORS["default"] if "default" in cfg else COLORS["tuned"]

        plt.bar(x + offset, vals, width, color=color, alpha=0.9, label=labels[cfg])

        for j, v in enumerate(vals):
            plt.text(x[j] + offset, v + 0.04, f"{v:.2f}", ha="center", va="bottom", fontsize=8)

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.xlabel("Lighting condition")
    plt.ylabel("ATE RMSE [m]")
    plt.title("ORB-SLAM3 parameter ablation, t03")
    plt.grid(axis="y", alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig3_orbslam3_ablation_t03_default_vs_tuned_ate")

def plot_lowlight_t03_ablation_ate_time():
    lighting = "lowlight"

    default_csv = ABL_ALIGNED / f"{lighting}_t03_orbslam3_default_aligned.csv"
    tuned_csv = MAIN_ALIGNED / f"{lighting}_t03_orbslam3_aligned.csv"

    default = pd.read_csv(default_csv)
    tuned = pd.read_csv(tuned_csv)

    t_end = min(float(default["t_s"].max()), float(tuned["t_s"].max()))
    default = default[default["t_s"] <= t_end].reset_index(drop=True)
    tuned = tuned[tuned["t_s"] <= t_end].reset_index(drop=True)

    plt.figure(figsize=(7.2, 4.0))

    plt.plot(
        default["t_s"],
        ate_series(default),
        color=COLORS["default"],
        linewidth=1.6,
        label="Default 1000/20/7",
    )

    plt.plot(
        tuned["t_s"],
        ate_series(tuned),
        color=COLORS["tuned"],
        linewidth=1.6,
        label="Tuned 1800/10/4",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("ATE [m]")
    plt.title("Lowlight t03 ORB-SLAM3 ablation")
    plt.grid(alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig4_lowlight_t03_orbslam3_ablation_ate_time")

def plot_lowlight_t03_ablation_overlay():
    lighting = "lowlight"

    default_csv = ABL_ALIGNED / f"{lighting}_t03_orbslam3_default_aligned.csv"
    tuned_csv = MAIN_ALIGNED / f"{lighting}_t03_orbslam3_aligned.csv"

    default = pd.read_csv(default_csv)
    tuned = pd.read_csv(tuned_csv)

    plt.figure(figsize=(5.6, 5.2))

    plt.plot(tuned["gt_x"], tuned["gt_y"], color=COLORS["gt"], linewidth=2.2, label="GT")
    plt.plot(default["est_x_al"], default["est_y_al"], color=COLORS["default"], linewidth=1.6, label="Default 1000/20/7")
    plt.plot(tuned["est_x_al"], tuned["est_y_al"], color=COLORS["tuned"], linewidth=1.6, label="Tuned 1800/10/4")

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Lowlight t03 ORB-SLAM3 ablation overlay")
    plt.axis("equal")
    plt.grid(alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig5_lowlight_t03_orbslam3_ablation_overlay")

def make_final_tables():
    if MAIN_TABLE.exists():
        main = pd.read_csv(MAIN_TABLE)
        main.to_csv(FINAL / "table1_main_v6_lighting_mean_std.csv", index=False)

    abl = pd.read_csv(ABL_TABLE)
    abl.to_csv(FINAL / "table2_orbslam3_default_vs_tuned_t03.csv", index=False)

    rows = []
    for lighting in LIGHTINGS:
        d = abl[(abl["lighting"] == lighting) & (abl["config"] == "default_1000_20_7")].iloc[0]
        t = abl[(abl["lighting"] == lighting) & (abl["config"] == "tuned_1800_10_4")].iloc[0]

        def imp(a, b):
            return 100.0 * (float(a) - float(b)) / float(a) if abs(float(a)) > 1e-12 else np.nan

        rows.append({
            "lighting": lighting,
            "trial": "t03",
            "default_ate_rmse_m": float(d["ate_rmse_m"]),
            "tuned_ate_rmse_m": float(t["ate_rmse_m"]),
            "tuned_ate_improvement_pct": imp(d["ate_rmse_m"], t["ate_rmse_m"]),
            "default_norm_ate": float(d["ate_norm_rmse"]),
            "tuned_norm_ate": float(t["ate_norm_rmse"]),
            "tuned_norm_ate_improvement_pct": imp(d["ate_norm_rmse"], t["ate_norm_rmse"]),
            "default_yaw_mean_deg": float(d["yaw_abs_mean_deg"]),
            "tuned_yaw_mean_deg": float(t["yaw_abs_mean_deg"]),
            "tuned_yaw_improvement_pct": imp(d["yaw_abs_mean_deg"], t["yaw_abs_mean_deg"]),
            "default_pose_output_ratio": float(d["pose_count_ratio"]),
            "tuned_pose_output_ratio": float(t["pose_count_ratio"]),
        })

    effect = pd.DataFrame(rows)
    effect.to_csv(FINAL / "table3_orbslam3_ablation_t03_effectiveness.csv", index=False)

    print(f"Saved final tables in: {FINAL}")

def main():
    plot_main_normalized_ate()
    plot_lowlight_t03_main_overlay()
    plot_ablation_t03_bars()
    plot_lowlight_t03_ablation_ate_time()
    plot_lowlight_t03_ablation_overlay()
    make_final_tables()

    print(f"\nPaper figures saved in: {OUT}")

if __name__ == "__main__":
    main()
