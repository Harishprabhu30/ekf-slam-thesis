import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

PHASE_NAME = {
    0: "stop",
    1: "square",
    2: "straight",
    3: "cw_rotation",
    4: "curve"
}

def load_corner_segments(phase_csv: str, t_end_s: float, include_phases=[1,4]):
    """
    Load all corner segments (square + curve) from phase CSV.
    Returns list of tuples: (phase_id, t0_s, t1_s)
    """
    phases = pd.read_csv(phase_csv).sort_values("t_ns").reset_index(drop=True)
    if len(phases[phases["phase"]==1])==0:
        raise RuntimeError("No phase==1 found in phase CSV.")
    t0_anchor_ns = int(phases[phases["phase"]==1].iloc[0]["t_ns"])
    phases["t_s"] = (phases["t_ns"].astype(np.int64)-t0_anchor_ns)*1e-9
    phases = phases.sort_values("t_s").reset_index(drop=True)
    phases = phases.drop_duplicates(subset=["phase"], keep="first")

    segs = []
    for i, row in phases.iterrows():
        ph = int(row["phase"])
        if ph not in include_phases:
            continue
        t0 = float(row["t_s"])
        t1 = float(phases.iloc[i+1]["t_s"]) if i+1 < len(phases) else t_end_s
        t0c = max(0.0, min(t0, t_end_s))
        t1c = max(0.0, min(t1, t_end_s))
        if t1c > t0c:
            segs.append((ph, t0c, t1c))
    return segs

if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--wheel", required=True)
    ap.add_argument("--ekf", required=True)
    ap.add_argument("--phase_csv", required=True)
    ap.add_argument("--out", default=None, help="Output PNG file. Defaults to same folder as GT CSV")
    args = ap.parse_args()

    # Default output path: same folder as GT CSV
    if args.out is None:
        folder = os.path.dirname(args.gt)
        args.out = os.path.join(folder, "corner_zoom_all.png")

    # Load trajectories
    gt_df = pd.read_csv(args.gt)
    wheel_df = pd.read_csv(args.wheel)
    ekf_df = pd.read_csv(args.ekf)

    if "t_s" not in gt_df.columns:
        gt_df["t_s"] = (gt_df["t_ns"] - gt_df["t_ns"].iloc[0])*1e-9

    if "gt_x" not in gt_df.columns:
        gt_df = gt_df.rename(columns={"x":"gt_x","y":"gt_y","yaw":"gt_yaw"})

    t_end = min(gt_df["t_s"].max(), wheel_df["t_s"].max(), ekf_df["t_s"].max())
    corner_segs = load_corner_segments(args.phase_csv, t_end_s=t_end)

    n_turns = len(corner_segs)
    fig, axes = plt.subplots(1, n_turns, figsize=(4*n_turns,4), squeeze=False)

    for i, (ph, t0, t1) in enumerate(corner_segs):
        ax = axes[0,i]
        gt_seg = gt_df[(gt_df["t_s"]>=t0) & (gt_df["t_s"]<=t1)]
        wheel_seg = wheel_df[(wheel_df["t_s"]>=t0) & (wheel_df["t_s"]<=t1)]
        ekf_seg = ekf_df[(ekf_df["t_s"]>=t0) & (ekf_df["t_s"]<=t1)]

        ax.plot(gt_seg["gt_x"], gt_seg["gt_y"], 'k-', label="GT", linewidth=1.5)
        ax.plot(wheel_seg["est_x_al"], wheel_seg["est_y_al"], 'r--', label="Wheel", alpha=0.8)
        ax.plot(ekf_seg["est_x_al"], ekf_seg["est_y_al"], 'b-.', label="EKF", alpha=0.8)

        ax.set_title(f"{PHASE_NAME.get(ph,'?')} {i+1}\n[{t0:.1f}-{t1:.1f}s]")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.axis('equal')
        ax.grid(True, linestyle="--", alpha=0.5)
        if i==0:
            ax.legend()

    plt.tight_layout()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=300)
    print(f"Saved corner zoom figure: {args.out}")
    plt.show()
