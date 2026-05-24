from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

METRICS = ROOT / "metrics" / "v6_all_trial_metrics.csv"
ALIGNED = ROOT / "aligned"

ABL_ROOT = ROOT / "ablations" / "default_vs_tuned_t01"
ABL_TABLE = ABL_ROOT / "tables" / "orbslam3_default_vs_tuned_t01.csv"
ABL_ALIGNED = ABL_ROOT / "aligned"

OUT = ROOT / "paper_figures"
OUT.mkdir(parents=True, exist_ok=True)

LIGHTINGS = ["bright", "dim", "lowlight"]
ESTIMATORS = ["wheel", "ekf", "orbslam3"]

LABELS = {
    "wheel": "Wheel",
    "ekf": "EKF",
    "orbslam3": "ORB-SLAM3",
}

COLORS = {
    "gt": "black",
    "wheel": "tab:blue",
    "ekf": "tab:orange",
    "orbslam3": "tab:red",
    "default": "tab:blue",
    "tuned": "tab:red",
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

def yaw_error_deg(df):
    err = df["gt_yaw"].values - df["est_yaw_al"].values
    err = np.array([wrap_pi(a) for a in err])
    return np.degrees(err)

def load_aligned(lighting, trial, estimator):
    p = ALIGNED / f"{lighting}_{trial}_{estimator}_aligned.csv"
    if not p.exists():
        raise FileNotFoundError(p)
    return pd.read_csv(p)

def plot_metric_trial_points(df, metric, ylabel, title, name):
    x = np.arange(len(LIGHTINGS))
    width = 0.24

    plt.figure(figsize=(7.2, 4.2))

    for i, est in enumerate(ESTIMATORS):
        offset = (i - 1) * width
        means = []
        stds = []

        for j, lighting in enumerate(LIGHTINGS):
            g = df[(df["lighting"] == lighting) & (df["estimator"] == est)]
            vals = pd.to_numeric(g[metric], errors="coerce").dropna().values

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
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(axis="y", alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig(name)

def plot_orb_only_trial_points(df):
    orb = df[df["estimator"] == "orbslam3"].copy()
    x = np.arange(len(LIGHTINGS))

    plt.figure(figsize=(5.8, 4.0))

    means = []
    stds = []

    for j, lighting in enumerate(LIGHTINGS):
        g = orb[orb["lighting"] == lighting]
        vals = pd.to_numeric(g["ate_rmse_m"], errors="coerce").dropna().values

        means.append(float(np.mean(vals)))
        stds.append(float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0)

        jitter = np.linspace(-0.05, 0.05, len(vals))
        plt.scatter(
            np.full(len(vals), x[j]) + jitter,
            vals,
            color=COLORS["orbslam3"],
            edgecolor="black",
            linewidth=0.35,
            s=38,
            alpha=0.9,
            zorder=3,
        )

    plt.errorbar(
        x,
        means,
        yerr=stds,
        fmt="o",
        color=COLORS["orbslam3"],
        capsize=4,
        markersize=7,
        linewidth=1.5,
        label="Mean ± std",
        zorder=4,
    )

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.xlabel("Lighting condition")
    plt.ylabel("ATE RMSE [m]")
    plt.title("ORB-SLAM3 error across lighting")
    plt.grid(axis="y", alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig_orbslam3_ate_trials_mean_std")

def plot_representative_overlay(lighting="lowlight", trial="t01"):
    dfs = {est: load_aligned(lighting, trial, est) for est in ESTIMATORS}

    plt.figure(figsize=(5.6, 5.2))

    gt = dfs["wheel"]
    plt.plot(
        gt["gt_x"], gt["gt_y"],
        color=COLORS["gt"],
        linewidth=2.2,
        label="GT",
    )

    for est in ESTIMATORS:
        df = dfs[est]
        plt.plot(
            df["est_x_al"], df["est_y_al"],
            color=COLORS[est],
            linewidth=1.6,
            alpha=0.92,
            label=LABELS[est],
        )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"SE(2)-aligned trajectory, {lighting} {trial}")
    plt.axis("equal")
    plt.grid(alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig(f"fig_trajectory_overlay_{lighting}_{trial}_se2_aligned")

def plot_ablation_bars():
    if not ABL_TABLE.exists():
        print(f"Skipping ablation bars; missing {ABL_TABLE}")
        return

    df = pd.read_csv(ABL_TABLE)

    configs = ["default_1000_20_7", "tuned_1800_10_4"]
    cfg_labels = {
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

        plt.bar(
            x + offset,
            vals,
            width,
            color=color,
            alpha=0.9,
            label=cfg_labels[cfg],
        )

        for j, v in enumerate(vals):
            plt.text(
                x[j] + offset,
                v + 0.04,
                f"{v:.2f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.xlabel("Lighting condition")
    plt.ylabel("ATE RMSE [m]")
    plt.title("ORB-SLAM3 parameter ablation, t01")
    plt.grid(axis="y", alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig_orbslam3_ablation_default_vs_tuned_ate")

def plot_ablation_lowlight_ate_time():
    lighting = "lowlight"

    default_csv = ABL_ALIGNED / f"{lighting}_t01_orbslam3_default_aligned.csv"
    tuned_csv = ALIGNED / f"{lighting}_t01_orbslam3_aligned.csv"

    if not default_csv.exists() or not tuned_csv.exists():
        print("Skipping ablation ATE-time plot; aligned files missing.")
        return

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
    plt.title("Low-light ORB-SLAM3 ablation, t01")
    plt.grid(alpha=0.3)
    plt.legend(frameon=True)
    plt.tight_layout()
    savefig("fig_lowlight_t01_orbslam3_ablation_ate_time")

def main():
    if not METRICS.exists():
        raise FileNotFoundError(METRICS)

    df = pd.read_csv(METRICS)

    plot_metric_trial_points(
        df,
        metric="ate_norm_rmse",
        ylabel="Normalized ATE RMSE [m/m]",
        title="Normalized trajectory error across repeated trials",
        name="fig_main_normalized_ate_trials_mean_std",
    )

    plot_metric_trial_points(
        df,
        metric="ate_rmse_m",
        ylabel="ATE RMSE [m]",
        title="ATE RMSE across repeated trials",
        name="fig_main_ate_trials_mean_std",
    )

    plot_orb_only_trial_points(df)

    # Use lowlight t01 because it explains the surprising ORB result.
    plot_representative_overlay("lowlight", "t01")

    plot_ablation_bars()
    plot_ablation_lowlight_ate_time()

    print(f"\nPaper figures saved in: {OUT}")

if __name__ == "__main__":
    main()
