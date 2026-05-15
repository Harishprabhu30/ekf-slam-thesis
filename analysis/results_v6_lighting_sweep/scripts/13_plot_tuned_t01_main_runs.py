from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

ALIGNED = ROOT / "aligned"
PLOTS = ROOT / "plots" / "tuned_t01_main"

PLOTS.mkdir(parents=True, exist_ok=True)

LIGHTINGS = ["bright", "dim", "lowlight"]
ESTIMATORS = ["wheel", "ekf", "orbslam3"]

COLORS = {
    "gt": "black",
    "wheel": "tab:blue",
    "ekf": "tab:orange",
    "orbslam3": "tab:red",
}

LABELS = {
    "wheel": "Wheel",
    "ekf": "EKF",
    "orbslam3": "ORB-SLAM3 tuned 1800/10/4",
}

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def load_aligned(lighting, estimator):
    p = ALIGNED / f"{lighting}_t01_{estimator}_aligned.csv"
    if not p.exists():
        raise FileNotFoundError(p)
    return pd.read_csv(p)

def ate_series(df):
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    return np.sqrt(ex * ex + ey * ey)

def yaw_error_deg(df):
    err = df["gt_yaw"].values - df["est_yaw_al"].values
    err = np.array([wrap_pi(a) for a in err])
    return np.degrees(err)

def cumulative_gt_distance(df):
    x = df["gt_x"].values
    y = df["gt_y"].values

    dx = np.diff(x, prepend=x[0])
    dy = np.diff(y, prepend=y[0])

    return np.cumsum(np.sqrt(dx * dx + dy * dy))

def drift_rate_series(df, min_dist=0.2):
    ate = ate_series(df)
    dist = cumulative_gt_distance(df)

    drift = np.full_like(ate, np.nan, dtype=float)
    valid = dist >= min_dist
    drift[valid] = ate[valid] / dist[valid]

    return drift

def common_trim(dfs):
    t_end = min(float(df["t_s"].max()) for df in dfs.values())

    out = {}
    for k, df in dfs.items():
        out[k] = df[df["t_s"] <= t_end].reset_index(drop=True)

    return out, t_end

def plot_trajectory_overlay(lighting):
    dfs = {est: load_aligned(lighting, est) for est in ESTIMATORS}
    dfs, _ = common_trim(dfs)

    out = PLOTS / f"{lighting}_t01_tuned_trajectory_overlay.png"

    plt.figure(figsize=(8, 8))

    # GT from wheel file
    w = dfs["wheel"]
    plt.plot(
        w["gt_x"], w["gt_y"],
        color=COLORS["gt"],
        linewidth=2.5,
        label="GT",
    )

    for est in ESTIMATORS:
        df = dfs[est]
        plt.plot(
            df["est_x_al"], df["est_y_al"],
            color=COLORS[est],
            linewidth=1.8,
            alpha=0.9,
            label=LABELS[est],
        )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"Tuned Main Trajectory Overlay — {lighting.capitalize()} t01")
    plt.axis("equal")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def plot_ate_vs_time(lighting):
    dfs = {est: load_aligned(lighting, est) for est in ESTIMATORS}
    dfs, _ = common_trim(dfs)

    out = PLOTS / f"{lighting}_t01_tuned_ate_vs_time.png"

    plt.figure(figsize=(12, 5))

    for est in ESTIMATORS:
        df = dfs[est]
        plt.plot(
            df["t_s"],
            ate_series(df),
            color=COLORS[est],
            linewidth=1.8,
            alpha=0.9,
            label=LABELS[est],
        )

    plt.xlabel("Time [s]")
    plt.ylabel("ATE [m]")
    plt.title(f"Tuned Main ATE vs Time — {lighting.capitalize()} t01")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def plot_drift_vs_time(lighting):
    dfs = {est: load_aligned(lighting, est) for est in ESTIMATORS}
    dfs, _ = common_trim(dfs)

    out = PLOTS / f"{lighting}_t01_tuned_drift_rate_vs_time.png"

    plt.figure(figsize=(12, 5))

    for est in ESTIMATORS:
        df = dfs[est]
        plt.plot(
            df["t_s"],
            drift_rate_series(df),
            color=COLORS[est],
            linewidth=1.8,
            alpha=0.9,
            label=LABELS[est],
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Cumulative drift rate [m/m]")
    plt.title(f"Tuned Main Drift Rate vs Time — {lighting.capitalize()} t01")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def plot_yaw_error_vs_time(lighting):
    dfs = {est: load_aligned(lighting, est) for est in ESTIMATORS}
    dfs, _ = common_trim(dfs)

    out = PLOTS / f"{lighting}_t01_tuned_yaw_error_vs_time.png"

    plt.figure(figsize=(12, 5))

    for est in ESTIMATORS:
        df = dfs[est]
        plt.plot(
            df["t_s"],
            yaw_error_deg(df),
            color=COLORS[est],
            linewidth=1.8,
            alpha=0.9,
            label=LABELS[est],
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Yaw error [deg]")
    plt.title(f"Tuned Main Yaw Error vs Time — {lighting.capitalize()} t01")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()
    print(f"Saved: {out}")

def main():
    for lighting in LIGHTINGS:
        print("\n" + "=" * 80)
        print(f"Plotting tuned main t01: {lighting}")
        print("=" * 80)

        plot_trajectory_overlay(lighting)
        plot_ate_vs_time(lighting)
        plot_drift_vs_time(lighting)
        plot_yaw_error_vs_time(lighting)

    print("\nDone.")
    print(f"Output folder: {PLOTS}")

if __name__ == "__main__":
    main()
