from pathlib import Path
import math
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

ALIGNED = ROOT / "aligned"
EXTRACTED = ROOT / "extracted"
METRICS = ROOT / "metrics" / "v6_all_trial_metrics.csv"

OUT_REP = ROOT / "plots" / "clean_representative_runs"
OUT_SUM = ROOT / "plots" / "clean_summary"

OUT_REP.mkdir(parents=True, exist_ok=True)
OUT_SUM.mkdir(parents=True, exist_ok=True)

COLORS = {
    "gt": "black",
    "wheel": "tab:blue",
    "ekf": "tab:orange",
    "orbslam3": "tab:red",
}

LABELS = {
    "wheel": "Wheel",
    "ekf": "EKF",
    "orbslam3": "ORB-SLAM3",
}

ESTIMATORS = ["wheel", "ekf", "orbslam3"]
LIGHTINGS = ["bright", "dim", "lowlight"]

PHASE_NAME = {
    0: "stop",
    1: "square",
    2: "straight",
    3: "cw_rotation",
    4: "curve",
}

SELECTED_RUNS = [
    ("bright", "t02"),
    ("dim", "t01"),
    ("lowlight", "t01"),
    ("lowlight", "t03"),
]

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

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

def compute_motion_t0_ns(traj_df, vel_thresh=0.02, sustain_s=0.5):
    t = traj_df["t_ns"].values.astype(np.int64)
    x = traj_df["x"].values
    y = traj_df["y"].values

    if len(t) < 3:
        return int(t[0])

    dt = np.diff(t) * 1e-9
    dx = np.diff(x)
    dy = np.diff(y)
    dt = np.maximum(dt, 1e-6)

    speed = np.sqrt(dx * dx + dy * dy) / dt
    mean_dt = float(np.median(dt)) if len(dt) else 0.05
    k = max(3, int(sustain_s / max(mean_dt, 1e-3)))

    if len(speed) < k:
        return int(t[0])

    ma = np.convolve(speed, np.ones(k) / k, mode="same")
    idx = int(np.argmax(ma > vel_thresh))

    if ma[idx] <= vel_thresh:
        return int(t[0])

    return int(t[idx])

def load_phase_times(lighting, trial):
    prefix = f"{lighting}_{trial}_wheel"

    phase_csv = EXTRACTED / f"{prefix}_phase_events.csv"
    gt_csv = EXTRACTED / f"{prefix}_gt.csv"

    if not phase_csv.exists() or not gt_csv.exists():
        return pd.DataFrame(columns=["t_s", "phase"])

    phases = pd.read_csv(phase_csv)
    gt = pd.read_csv(gt_csv)

    if phases.empty or gt.empty:
        return pd.DataFrame(columns=["t_s", "phase"])

    gt_t0 = compute_motion_t0_ns(gt)
    phases["t_s"] = (phases["t_ns"].astype(np.int64) - int(gt_t0)) * 1e-9

    phases = phases.sort_values("t_s").drop_duplicates(subset=["phase"], keep="first")
    phases = phases[(phases["t_s"] >= 0.0)].reset_index(drop=True)

    return phases[["t_s", "phase"]]

def add_phase_lines(phases, t_end):
    ymin, ymax = plt.ylim()

    for _, row in phases.iterrows():
        ts = float(row["t_s"])
        ph = int(row["phase"])

        if 0.0 <= ts <= t_end:
            plt.axvline(ts, color="gray", linestyle="--", linewidth=1, alpha=0.5)
            plt.text(
                ts + 0.25,
                ymin + 0.03 * (ymax - ymin),
                f"{ph}:{PHASE_NAME.get(ph, '?')}",
                rotation=90,
                va="bottom",
                fontsize=8,
                color="dimgray",
            )

def plot_trajectory_overlay(lighting, trial):
    dfs = {est: load_aligned(lighting, trial, est) for est in ESTIMATORS}

    plt.figure(figsize=(8, 8))

    w = dfs["wheel"]
    plt.plot(
        w["gt_x"], w["gt_y"],
        label="GT",
        color=COLORS["gt"],
        linewidth=2.4,
    )

    for est in ESTIMATORS:
        df = dfs[est]
        plt.plot(
            df["est_x_al"], df["est_y_al"],
            label=LABELS[est],
            color=COLORS[est],
            linewidth=1.8,
            alpha=0.9,
        )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"Trajectory Overlay — {lighting.capitalize()} {trial}")
    plt.axis("equal")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    out = OUT_REP / f"{lighting}_{trial}_trajectory_overlay_clean.png"
    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def plot_ate_vs_time(lighting, trial):
    dfs = {est: load_aligned(lighting, trial, est) for est in ESTIMATORS}
    phases = load_phase_times(lighting, trial)
    t_end = min(float(df["t_s"].max()) for df in dfs.values())

    plt.figure(figsize=(12, 5))

    for est in ESTIMATORS:
        df = dfs[est]
        df = df[df["t_s"] <= t_end].reset_index(drop=True)

        plt.plot(
            df["t_s"],
            ate_series(df),
            label=LABELS[est],
            color=COLORS[est],
            linewidth=1.8,
            alpha=0.9,
        )

    plt.xlabel("Time [s]")
    plt.ylabel("ATE [m]")
    plt.title(f"ATE vs Time — {lighting.capitalize()} {trial}")
    plt.grid(True, alpha=0.35)
    plt.legend()

    if not phases.empty:
        add_phase_lines(phases, t_end)

    plt.tight_layout()

    out = OUT_REP / f"{lighting}_{trial}_ate_vs_time_clean.png"
    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def plot_yaw_error_vs_time(lighting, trial):
    dfs = {est: load_aligned(lighting, trial, est) for est in ESTIMATORS}
    phases = load_phase_times(lighting, trial)
    t_end = min(float(df["t_s"].max()) for df in dfs.values())

    plt.figure(figsize=(12, 5))

    for est in ESTIMATORS:
        df = dfs[est]
        df = df[df["t_s"] <= t_end].reset_index(drop=True)

        plt.plot(
            df["t_s"],
            yaw_error_deg(df),
            label=LABELS[est],
            color=COLORS[est],
            linewidth=1.8,
            alpha=0.9,
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Yaw error [deg]")
    plt.title(f"Yaw Error vs Time — {lighting.capitalize()} {trial}")
    plt.grid(True, alpha=0.35)
    plt.legend()

    if not phases.empty:
        add_phase_lines(phases, t_end)

    plt.tight_layout()

    out = OUT_REP / f"{lighting}_{trial}_yaw_error_vs_time_clean.png"
    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def plot_metric_points(df, metric, ylabel, title, filename):
    x = np.arange(len(LIGHTINGS))
    width = 0.24

    plt.figure(figsize=(10, 5.2))

    for i, est in enumerate(ESTIMATORS):
        means = []
        stds = []

        offset = (i - 1) * width

        for j, lighting in enumerate(LIGHTINGS):
            g = df[(df["lighting"] == lighting) & (df["estimator"] == est)]
            vals = pd.to_numeric(g[metric], errors="coerce").dropna().values

            if len(vals) == 0:
                means.append(np.nan)
                stds.append(0.0)
                continue

            mean = float(np.mean(vals))
            std = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0

            means.append(mean)
            stds.append(std)

            # individual trial points
            jitter = np.linspace(-0.04, 0.04, len(vals)) if len(vals) > 1 else [0.0]
            plt.scatter(
                np.full(len(vals), x[j] + offset) + jitter,
                vals,
                color=COLORS[est],
                edgecolor="black",
                linewidth=0.4,
                s=45,
                alpha=0.85,
                zorder=3,
            )

        plt.errorbar(
            x + offset,
            means,
            yerr=stds,
            fmt="o",
            color=COLORS[est],
            capsize=5,
            markersize=8,
            linewidth=1.8,
            label=LABELS[est],
            zorder=4,
        )

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.ylabel(ylabel)
    plt.xlabel("Lighting condition")
    plt.title(title)
    plt.grid(axis="y", alpha=0.35)
    plt.legend()
    plt.tight_layout()

    out = OUT_SUM / filename
    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def plot_orb_metric_points(df, metric, ylabel, title, filename):
    orb = df[df["estimator"] == "orbslam3"].copy()
    x = np.arange(len(LIGHTINGS))

    plt.figure(figsize=(7, 5.2))

    means = []
    stds = []

    for j, lighting in enumerate(LIGHTINGS):
        g = orb[orb["lighting"] == lighting]
        vals = pd.to_numeric(g[metric], errors="coerce").dropna().values

        mean = float(np.mean(vals))
        std = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0

        means.append(mean)
        stds.append(std)

        jitter = np.linspace(-0.06, 0.06, len(vals)) if len(vals) > 1 else [0.0]
        plt.scatter(
            np.full(len(vals), x[j]) + jitter,
            vals,
            color=COLORS["orbslam3"],
            edgecolor="black",
            linewidth=0.4,
            s=55,
            alpha=0.85,
            zorder=3,
        )

    plt.errorbar(
        x,
        means,
        yerr=stds,
        fmt="o",
        color=COLORS["orbslam3"],
        capsize=5,
        markersize=9,
        linewidth=1.8,
        label="Mean ± std",
        zorder=4,
    )

    plt.xticks(x, [l.capitalize() for l in LIGHTINGS])
    plt.ylabel(ylabel)
    plt.xlabel("Lighting condition")
    plt.title(title)
    plt.grid(axis="y", alpha=0.35)
    plt.legend()
    plt.tight_layout()

    out = OUT_SUM / filename
    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def main():
    for lighting, trial in SELECTED_RUNS:
        plot_trajectory_overlay(lighting, trial)
        plot_ate_vs_time(lighting, trial)
        plot_yaw_error_vs_time(lighting, trial)

    df = pd.read_csv(METRICS)

    plot_metric_points(
        df,
        metric="ate_rmse_m",
        ylabel="ATE RMSE [m]",
        title="ATE RMSE Across Trials",
        filename="trial_points_ate_rmse.png",
    )

    plot_metric_points(
        df,
        metric="ate_norm_rmse",
        ylabel="Normalized ATE RMSE [m/m]",
        title="Normalized ATE RMSE Across Trials",
        filename="trial_points_normalized_ate.png",
    )

    plot_metric_points(
        df,
        metric="yaw_abs_mean_deg",
        ylabel="Mean Absolute Yaw Error [deg]",
        title="Yaw Error Across Trials",
        filename="trial_points_yaw_error.png",
    )

    plot_metric_points(
        df,
        metric="rpe_trans_rmse_m_1.0s",
        ylabel="RPE Translation RMSE @1s [m]",
        title="RPE Translation Across Trials",
        filename="trial_points_rpe_translation.png",
    )

    plot_orb_metric_points(
        df,
        metric="ate_rmse_m",
        ylabel="ATE RMSE [m]",
        title="ORB-SLAM3 ATE RMSE Across Lighting Conditions",
        filename="orb_trial_points_ate_rmse.png",
    )

    plot_orb_metric_points(
        df,
        metric="ate_norm_rmse",
        ylabel="Normalized ATE RMSE [m/m]",
        title="ORB-SLAM3 Normalized ATE Across Lighting Conditions",
        filename="orb_trial_points_normalized_ate.png",
    )

    print("\nDone clean final plots.")
    print(f"Representative plots: {OUT_REP}")
    print(f"Summary plots:        {OUT_SUM}")

if __name__ == "__main__":
    main()
