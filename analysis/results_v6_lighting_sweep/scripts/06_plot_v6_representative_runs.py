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
PLOTS = ROOT / "plots" / "representative_runs"

PLOTS.mkdir(parents=True, exist_ok=True)

PHASE_NAME = {
    0: "stop",
    1: "square",
    2: "straight",
    3: "cw_rotation",
    4: "curve",
}

ESTIMATORS = [
    ("wheel", "Wheel"),
    ("ekf", "EKF"),
    ("orbslam3", "ORB-SLAM3"),
]

# These are the most useful trials based on the validation table.
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
    # Use wheel GT/phase as reference for the trial.
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

def load_aligned(lighting, trial, estimator):
    p = ALIGNED / f"{lighting}_{trial}_{estimator}_aligned.csv"
    if not p.exists():
        raise FileNotFoundError(p)
    return pd.read_csv(p)

def common_t_end(dfs):
    return min(float(df["t_s"].max()) for df in dfs.values())

def add_phase_lines(phases, t_end):
    for _, row in phases.iterrows():
        ts = float(row["t_s"])
        ph = int(row["phase"])

        if 0.0 <= ts <= t_end:
            plt.axvline(ts, linestyle="--", linewidth=1, alpha=0.6)
            plt.text(
                ts + 0.25,
                plt.ylim()[0],
                f"{ph}:{PHASE_NAME.get(ph, '?')}",
                rotation=90,
                va="bottom",
                fontsize=8,
            )

def plot_trajectory_overlay(lighting, trial):
    dfs = {est: load_aligned(lighting, trial, est) for est, _ in ESTIMATORS}

    out = PLOTS / f"{lighting}_{trial}_trajectory_overlay.png"

    plt.figure(figsize=(8, 8))

    # GT from wheel-aligned file. GT is trial-specific.
    w = dfs["wheel"]
    plt.plot(w["gt_x"], w["gt_y"], label="GT", linewidth=2)

    for est, label in ESTIMATORS:
        df = dfs[est]
        plt.plot(df["est_x_al"], df["est_y_al"], label=label, alpha=0.85)

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"Trajectory Overlay — {lighting} {trial}")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def plot_ate_vs_time(lighting, trial):
    dfs = {est: load_aligned(lighting, trial, est) for est, _ in ESTIMATORS}
    phases = load_phase_times(lighting, trial)
    t_end = common_t_end(dfs)

    out = PLOTS / f"{lighting}_{trial}_ate_vs_time.png"

    plt.figure(figsize=(12, 5))

    for est, label in ESTIMATORS:
        df = dfs[est]
        df = df[df["t_s"] <= t_end].reset_index(drop=True)
        plt.plot(df["t_s"], ate_series(df), label=label, alpha=0.9)

    plt.xlabel("Time [s]")
    plt.ylabel("ATE [m]")
    plt.title(f"ATE vs Time — {lighting} {trial}")
    plt.grid(True)
    plt.legend()

    if not phases.empty:
        add_phase_lines(phases, t_end)

    plt.tight_layout()
    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def plot_yaw_error_vs_time(lighting, trial):
    dfs = {est: load_aligned(lighting, trial, est) for est, _ in ESTIMATORS}
    phases = load_phase_times(lighting, trial)
    t_end = common_t_end(dfs)

    out = PLOTS / f"{lighting}_{trial}_yaw_error_vs_time.png"

    plt.figure(figsize=(12, 5))

    for est, label in ESTIMATORS:
        df = dfs[est]
        df = df[df["t_s"] <= t_end].reset_index(drop=True)
        plt.plot(df["t_s"], yaw_error_deg(df), label=label, alpha=0.9)

    plt.xlabel("Time [s]")
    plt.ylabel("Yaw error [deg]")
    plt.title(f"Yaw Error vs Time — {lighting} {trial}")
    plt.grid(True)
    plt.legend()

    if not phases.empty:
        add_phase_lines(phases, t_end)

    plt.tight_layout()
    plt.savefig(out, dpi=300)
    plt.close()

    print(f"Saved: {out}")

def main():
    for lighting, trial in SELECTED_RUNS:
        print("\n" + "=" * 80)
        print(f"Plotting representative run: {lighting} {trial}")
        print("=" * 80)

        plot_trajectory_overlay(lighting, trial)
        plot_ate_vs_time(lighting, trial)
        plot_yaw_error_vs_time(lighting, trial)

    print("\nDone representative plots.")
    print(f"Output folder: {PLOTS}")

if __name__ == "__main__":
    main()
