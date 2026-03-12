import os
import math
import numpy as np
import pandas as pd

# Mapping from phase ID → human readable name
# These correspond to the markers used during rosbag recording
PHASE_NAME = {
    0: "stop",
    1: "square",
    2: "straight",
    3: "cw_rotation",
    # 4: "arc"
}


def wrap_pi(a: float) -> float:
    """
    Normalize angle to range [-pi, pi]
    """
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_phase_segments(phase_csv: str, t_end_s: float) -> list:
    """
    Build phase segments as:
        [(phase_id, t0_s, t1_s), ...]

    Segments are built from /traj_phase events recorded in rosbag.

    Important design choice:
    We anchor time to the FIRST occurrence of phase == 1
    instead of motion detection. This guarantees consistent
    phase boundaries and prevents negative timestamps.
    """

    phases = pd.read_csv(phase_csv).sort_values("t_ns").reset_index(drop=True)

    if phases.empty:
        raise RuntimeError(f"Phase CSV '{phase_csv}' is empty")

    # -----------------------------------------------------
    # Find the first occurrence of phase 1 (trajectory start)
    # -----------------------------------------------------
    phase1 = phases[phases["phase"] == 1]

    if len(phase1) == 0:
        raise RuntimeError("Phase 1 not found in phase CSV. Cannot anchor segments.")

    t0_anchor_ns = int(phase1.iloc[0]["t_ns"])

    # -----------------------------------------------------
    # Convert timestamps → seconds relative to phase 1
    # -----------------------------------------------------
    phases["t_s"] = (phases["t_ns"].astype(np.int64) - t0_anchor_ns) * 1e-9
    phases = phases.sort_values("t_s").reset_index(drop=True)

    # Remove duplicate phase markers if they exist
    phases = phases.drop_duplicates(subset=["phase"], keep="first").reset_index(drop=True)

    segs = []

    for i in range(len(phases)):

        ph = int(phases.loc[i, "phase"])
        t0 = float(phases.loc[i, "t_s"])

        # Next phase start defines the segment end
        if i + 1 < len(phases):
            t1 = float(phases.loc[i + 1, "t_s"])
        else:
            t1 = float(t_end_s)

        # Clamp segment bounds inside valid trajectory range
        t0c = max(0.0, min(t0, t_end_s))
        t1c = max(0.0, min(t1, t_end_s))

        if t1c > t0c:
            segs.append((ph, t0c, t1c))

    if not segs:
        print(f"Warning: no segments loaded from '{phase_csv}' within t_end={t_end_s:.3f}s")
    else:
        print(f"Loaded {len(segs)} segments from '{phase_csv}' (anchored at phase=1)")

    return segs


def compute_segment_metrics(df: pd.DataFrame) -> dict:
    """
    Compute trajectory error metrics for a segment.
    """

    if df.empty:
        return {
            "n": 0,
            "ate_rmse_m": float("nan"),
            "ate_mean_m": float("nan"),
            "ate_max_m": float("nan"),
            "yaw_abs_mean_rad": float("nan"),
            "yaw_abs_max_rad": float("nan"),
        }

    # -----------------------------------------------------
    # Absolute Trajectory Error (ATE)
    # -----------------------------------------------------
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    ate = np.sqrt(ex * ex + ey * ey)

    # -----------------------------------------------------
    # Yaw error
    # -----------------------------------------------------
    yaw_err = df["gt_yaw"].values - df["est_yaw_al"].values
    yaw_err = np.array([wrap_pi(a) for a in yaw_err])
    yaw_abs = np.abs(yaw_err)

    # Utility statistics
    def rmse(x): return float(np.sqrt(np.mean(x * x))) if len(x) else float("nan")
    def mean(x): return float(np.mean(x)) if len(x) else float("nan")
    def mx(x): return float(np.max(x)) if len(x) else float("nan")

    return {
        "n": int(len(df)),
        "ate_rmse_m": rmse(ate),
        "ate_mean_m": mean(ate),
        "ate_max_m": mx(ate),
        "yaw_abs_mean_rad": mean(yaw_abs),
        "yaw_abs_max_rad": mx(yaw_abs),
    }


def evaluate_by_phase(aligned_csv: str, segs: list, name: str, include_phases=None) -> pd.DataFrame:
    """
    Compute metrics for each phase segment for a given estimator.
    """

    df = pd.read_csv(aligned_csv)

    if df.empty:
        print(f"Warning: aligned CSV '{aligned_csv}' is empty")
        return pd.DataFrame()

    rows = []

    for ph, t0, t1 in segs:

        # Skip unwanted phases
        if include_phases is not None and ph not in include_phases:
            continue

        # Extract trajectory segment
        seg = df[(df["t_s"] >= t0) & (df["t_s"] < t1)]

        # Compute metrics
        m = compute_segment_metrics(seg)

        rows.append({
            "estimator": name,
            "phase": ph,
            "phase_name": PHASE_NAME.get(ph, "unknown"),
            "t0_s": t0,
            "t1_s": t1,
            "duration_s": float(t1 - t0),
            **m
        })

    if not rows:
        print(f"Warning: No segments included for estimator '{name}'")

    return pd.DataFrame(rows)


if __name__ == "__main__":

    import argparse

    ap = argparse.ArgumentParser()

    # Input aligned trajectories
    ap.add_argument("--wheel", default="analysis/results_gt_traj_v3/wheel_synced_aligned.csv")
    ap.add_argument("--ekf", default="analysis/results_gt_traj_v3/ekf_synced_aligned.csv")

    # Phase event file extracted from GT bag
    ap.add_argument("--phase_csv", default="analysis/results_gt_traj_v3/gt_traj_phase_events.csv")

    # Output file
    ap.add_argument("--out", default="analysis/results_gt_traj_v3/phase_metrics.csv")

    # Phases to include
    ap.add_argument(
        "--include",
        default="1,2,3",
        help="Comma-separated phase ids to include (default: 1,2,3). Use '0,1,2,3' to include stop."
    )

    args = ap.parse_args()

    include_phases = [int(x.strip()) for x in args.include.split(",") if x.strip() != ""]
    include_set = set(include_phases)

    # -----------------------------------------------------
    # Determine common trajectory duration
    # (ensures fair comparison between estimators)
    # -----------------------------------------------------
    wheel_df = pd.read_csv(args.wheel)
    ekf_df = pd.read_csv(args.ekf)

    t_end = min(float(wheel_df["t_s"].max()), float(ekf_df["t_s"].max()))

    print(f"Common trajectory end time: {t_end:.3f}s")

    # -----------------------------------------------------
    # Load phase segments
    # -----------------------------------------------------
    segs = load_phase_segments(args.phase_csv, t_end_s=t_end)

    # Print segment boundaries for sanity check
    for ph, t0, t1 in segs:
        print(f"Phase {ph:>2} ({PHASE_NAME.get(ph,'unknown'):>10}): "
              f"{t0:8.2f}s -> {t1:8.2f}s | dur={t1-t0:6.2f}s")

    # -----------------------------------------------------
    # Compute metrics for both estimators
    # -----------------------------------------------------
    out_w = evaluate_by_phase(args.wheel, segs, "wheel", include_phases=include_set)
    out_e = evaluate_by_phase(args.ekf, segs, "ekf", include_phases=include_set)

    out = pd.concat([out_w, out_e], ignore_index=True)

    # Save results
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    out.to_csv(args.out, index=False)

    if out.empty:
        print("Warning: no metrics to report. Check phase CSV and aligned trajectories.")
    else:
        out = out.sort_values(["phase", "estimator"])

        # Display full set of metrics
        cols = [
            "phase",
            "phase_name",
            "estimator",
            "duration_s",
            "n",
            "ate_rmse_m",
            "ate_mean_m",
            "ate_max_m",
            "yaw_abs_mean_rad",
            "yaw_abs_max_rad"
        ]

        print("\n" + out[cols].to_string(index=False))

    print(f"\nSaved: {args.out}")
