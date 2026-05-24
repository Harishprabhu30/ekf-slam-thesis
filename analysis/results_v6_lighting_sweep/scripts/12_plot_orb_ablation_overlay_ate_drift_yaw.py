from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

MAIN_ALIGNED = ROOT / "aligned"

ABL_ROOT = ROOT / "ablations" / "default_vs_tuned_t01"
ABL_ALIGNED = ABL_ROOT / "aligned"
PLOTS = ABL_ROOT / "plots" / "trajectory_ate_drift_yaw"

PLOTS.mkdir(parents=True, exist_ok=True)

LIGHTINGS = ["bright", "dim", "lowlight"]

COLORS = {
    "gt": "black",
    "default": "tab:blue",
    "tuned": "tab:red",
}

LABELS = {
    "default": "Default ORB 1000/20/7",
    "tuned": "Tuned ORB 1800/10/4",
}

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def load_pair(lighting):
    default_csv = ABL_ALIGNED / f"{lighting}_t01_orbslam3_default_aligned.csv"
    tuned_csv = MAIN_ALIGNED / f"{lighting}_t01_orbslam3_aligned.csv"

    if not default_csv.exists():
        raise FileNotFoundError(default_csv)
    if not tuned_csv.exists():
        raise FileNotFoundError(tuned_csv)

    default = pd.read_csv(default_csv)
    tuned = pd.read_csv(tuned_csv)

    return default, tuned

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

    step = np.sqrt(dx * dx + dy * dy)
    return np.cumsum(step)

def drift_rate_series(df, min_dist=0.2):
    ate = ate_series(df)
    dist = cumulative_gt_distance(df)

    drift = np.full_like(ate, np.nan, dtype=float)
    valid = dist >= min_dist
    drift[valid] = ate[valid] / dist[valid]

    return drift

def common_time_trim(default, tuned):
    t_end = min(float(default["t_s"].max()), float(tuned["t_s"].max()))

    default = default[default["t_s"] <= t_end].reset_index(drop=True)
    tuned = tuned[tuned["t_s"] <= t_end].reset_index(drop=True)

    return default, tuned, t_end

def plot_trajectory_overlay(lighting):
    default, tuned = load_pair(lighting)
    default, tuned, _ = common_time_trim(default, tuned)

    out = PLOTS / f"{lighting}_t01_ablation_trajectory_overlay.png"

    plt.figure(figsize=(8, 8))

    # GT from tuned file. Same trial GT, separate experiment replay.
    plt.plot(
        tuned["gt_x"], tuned["gt_y"],
        color=COLORS["gt"],
        linewidth=2.4,
        label="GT",
    )

    plt.plot(
        default["est_x_al"], default["est_y_al"],
        color=COLORS["default"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["default"],
    )

    plt.plot(
        tuned["est_x_al"], tuned["est_y_al"],
        color=COLORS["tuned"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["tuned"],
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"ORB-SLAM3 Ablation Trajectory Overlay — {lighting.capitalize()} t01")
    plt.axis("equal")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def plot_ate_vs_time(lighting):
    default, tuned = load_pair(lighting)
    default, tuned, _ = common_time_trim(default, tuned)

    out = PLOTS / f"{lighting}_t01_ablation_ate_vs_time.png"

    plt.figure(figsize=(12, 5))

    plt.plot(
        default["t_s"],
        ate_series(default),
        color=COLORS["default"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["default"],
    )

    plt.plot(
        tuned["t_s"],
        ate_series(tuned),
        color=COLORS["tuned"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["tuned"],
    )

    plt.xlabel("Time [s]")
    plt.ylabel("ATE [m]")
    plt.title(f"ORB-SLAM3 Ablation ATE vs Time — {lighting.capitalize()} t01")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def plot_drift_vs_time(lighting):
    default, tuned = load_pair(lighting)
    default, tuned, _ = common_time_trim(default, tuned)

    out = PLOTS / f"{lighting}_t01_ablation_drift_rate_vs_time.png"

    plt.figure(figsize=(12, 5))

    plt.plot(
        default["t_s"],
        drift_rate_series(default),
        color=COLORS["default"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["default"],
    )

    plt.plot(
        tuned["t_s"],
        drift_rate_series(tuned),
        color=COLORS["tuned"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["tuned"],
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Cumulative drift rate [m/m]")
    plt.title(f"ORB-SLAM3 Ablation Drift Rate vs Time — {lighting.capitalize()} t01")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def plot_yaw_error_vs_time(lighting):
    default, tuned = load_pair(lighting)
    default, tuned, _ = common_time_trim(default, tuned)

    out = PLOTS / f"{lighting}_t01_ablation_yaw_error_vs_time.png"

    plt.figure(figsize=(12, 5))

    plt.plot(
        default["t_s"],
        yaw_error_deg(default),
        color=COLORS["default"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["default"],
    )

    plt.plot(
        tuned["t_s"],
        yaw_error_deg(tuned),
        color=COLORS["tuned"],
        linewidth=1.8,
        alpha=0.9,
        label=LABELS["tuned"],
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Yaw error [deg]")
    plt.title(f"ORB-SLAM3 Ablation Yaw Error vs Time — {lighting.capitalize()} t01")
    plt.grid(True, alpha=0.35)
    plt.legend()
    plt.tight_layout()

    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def main():
    for lighting in LIGHTINGS:
        print("\n" + "=" * 80)
        print(f"Plotting ORB ablation for {lighting} t01")
        print("=" * 80)

        plot_trajectory_overlay(lighting)
        plot_ate_vs_time(lighting)
        plot_drift_vs_time(lighting)
        plot_yaw_error_vs_time(lighting)

    print("\nDone.")
    print(f"Output folder: {PLOTS}")

if __name__ == "__main__":
    main()
