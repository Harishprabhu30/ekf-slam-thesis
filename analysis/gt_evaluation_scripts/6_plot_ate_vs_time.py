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

def load_phase_boundaries_phase1_anchor(phase_csv: str) -> pd.DataFrame:
    """
    Convert phase event record-time (t_ns) to aligned seconds t_s using phase==1 as t0.
    This matches your phase segmentation logic.
    """
    phases = pd.read_csv(phase_csv).sort_values("t_ns").reset_index(drop=True)
    if phases.empty:
        raise RuntimeError(f"Phase CSV '{phase_csv}' is empty")

    phase1 = phases[phases["phase"] == 1]
    if len(phase1) == 0:
        raise RuntimeError("Phase 1 not found in phase CSV. Cannot anchor boundaries.")

    t0_anchor_ns = int(phase1.iloc[0]["t_ns"])
    phases["t_s"] = (phases["t_ns"].astype(np.int64) - t0_anchor_ns) * 1e-9
    phases = phases.sort_values("t_s").reset_index(drop=True)

    # keep first occurrence of each phase (avoid duplicates)
    phases = phases.drop_duplicates(subset=["phase"], keep="first").reset_index(drop=True)
    return phases[["t_s", "phase"]]

def ate_series(df: pd.DataFrame) -> np.ndarray:
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    return np.sqrt(ex * ex + ey * ey)

if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--wheel", default="analysis/results_gt/wheel_synced_aligned.csv")
    ap.add_argument("--ekf", default="analysis/results_gt/ekf_synced_aligned.csv")
    ap.add_argument("--phase_csv", default="analysis/results_gt/traj_phase_events.csv")
    ap.add_argument("--out", default="analysis/results_gt/ate_vs_time_with_phases.png")
    ap.add_argument("--smooth_window", type=int, default=25,
                    help="Moving average window (samples). Set 1 for no smoothing.")
    args = ap.parse_args()

    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)
    phases = load_phase_boundaries_phase1_anchor(args.phase_csv)

    # Common time horizon
    t_end = min(float(wheel["t_s"].max()), float(ekf["t_s"].max()))
    wheel = wheel[wheel["t_s"] <= t_end].reset_index(drop=True)
    ekf = ekf[ekf["t_s"] <= t_end].reset_index(drop=True)

    w_ate = ate_series(wheel)
    e_ate = ate_series(ekf)

    # Optional smoothing
    def smooth(x, k):
        if k <= 1:
            return x
        k = int(k)
        return np.convolve(x, np.ones(k) / k, mode="same")

    w_plot = smooth(w_ate, args.smooth_window)
    e_plot = smooth(e_ate, args.smooth_window)

    plt.figure(figsize=(12, 5))
    plt.plot(wheel["t_s"], w_plot, label="Wheel ATE (aligned)")
    plt.plot(ekf["t_s"], e_plot, label="EKF ATE (aligned)", alpha=0.9)

    # Phase boundaries + labels
    for _, row in phases.iterrows():
        ts = float(row["t_s"])
        ph = int(row["phase"])
        if 0 <= ts <= t_end:
            plt.axvline(ts, linestyle="--", linewidth=1)
            plt.text(ts + 0.5, 0.02, f"{ph}:{PHASE_NAME.get(ph,'?')}",
                     rotation=90, va="bottom")

    plt.xlabel("Time [s] (anchored at phase=1)")
    plt.ylabel("ATE [m]")
    plt.title("ATE vs Time with Phase Boundaries (GT reference)")
    plt.grid(True)
    plt.legend()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.tight_layout()
    plt.savefig(args.out, dpi=300)
    print(f"Saved: {args.out}")
    plt.show()
