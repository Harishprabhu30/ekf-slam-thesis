import os
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


PHASE_NAME = {
    0: "stop",
    1: "square",
    2: "straight",
    3: "cw_rotation",
    #4: "arc",
}


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def compute_motion_t0_ns(traj_df: pd.DataFrame, vel_thresh=0.02, sustain_s=0.5) -> int:
    """Same motion-onset detector used in sync_and_align_v2."""
    t = traj_df["t_ns"].values.astype(np.int64)
    x = traj_df["x"].values
    y = traj_df["y"].values

    dt = np.diff(t) * 1e-9
    dx = np.diff(x)
    dy = np.diff(y)
    dt = np.maximum(dt, 1e-6)
    speed = np.sqrt(dx * dx + dy * dy) / dt

    if len(speed) < 10:
        return int(t[0])

    mean_dt = float(np.median(dt))
    k = max(3, int(sustain_s / max(mean_dt, 1e-3)))
    if len(speed) < k:
        return int(t[0])

    ma = np.convolve(speed, np.ones(k) / k, mode="same")
    idx = np.argmax(ma > vel_thresh)
    if ma[idx] <= vel_thresh:
        return int(t[0])
    return int(t[idx])


def load_phase_boundaries_as_ts(phase_csv: str, gt_traj_csv: str) -> pd.DataFrame:
    """
    Convert phase event record-time (t_ns) into t_s aligned timeline by subtracting GT motion t0.
    """
    phases = pd.read_csv(phase_csv)
    gt = pd.read_csv(gt_traj_csv)

    gt_t0 = compute_motion_t0_ns(gt)  # must match how you anchored t_s in sync
    phases["t_s"] = (phases["t_ns"].astype(np.int64) - int(gt_t0)) * 1e-9
    phases = phases.sort_values("t_s").reset_index(drop=True)
    return phases[["t_s", "phase"]]


def yaw_error_series(df_aligned: pd.DataFrame) -> np.ndarray:
    err = df_aligned["gt_yaw"].values - df_aligned["est_yaw_al"].values
    return np.array([wrap_pi(a) for a in err])


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--wheel", default="analysis/results_gt_traj_v3/wheel_synced_aligned.csv")
    ap.add_argument("--ekf", default="analysis/results_gt_traj_v3/ekf_synced_aligned.csv")
    ap.add_argument("--phase_csv", default="analysis/results_gt_traj_v3/gt_traj_phase_events.csv")
    ap.add_argument("--gt_traj_csv", default="analysis/results_gt_traj_v3/gt_traj.csv")
    ap.add_argument("--out", default="analysis/results_gt_traj_v3/yaw_error_vs_time.png")
    args = ap.parse_args()

    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)
    phases = load_phase_boundaries_as_ts(args.phase_csv, args.gt_traj_csv)

    # Ensure common time window for clean overlay
    t_end = min(float(wheel["t_s"].max()), float(ekf["t_s"].max()))
    wheel = wheel[wheel["t_s"] <= t_end].reset_index(drop=True)
    ekf = ekf[ekf["t_s"] <= t_end].reset_index(drop=True)

    w_err = yaw_error_series(wheel)
    e_err = yaw_error_series(ekf)

    plt.figure(figsize=(12, 5))
    plt.plot(wheel["t_s"], w_err, label="Wheel yaw error (GT - est)")
    plt.plot(ekf["t_s"], e_err, label="EKF yaw error (GT - est)", alpha=0.9)

    # Phase boundaries
    for _, row in phases.iterrows():
        ts = float(row["t_s"])
        if 0 <= ts <= t_end:
            ph = int(row["phase"])
            plt.axvline(ts, linestyle="--", linewidth=1)
            plt.text(ts + 0.2, 0.0, f"{ph}:{PHASE_NAME.get(ph,'?')}", rotation=90, va="bottom")

    plt.xlabel("Time [s] (aligned)")
    plt.ylabel("Yaw error [rad]")
    plt.title("Yaw Error vs Time with Phase Boundaries")
    plt.grid(True)
    plt.legend()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=300)
    print(f"Saved: {args.out}")
    plt.show()
