import os
import math
import numpy as np
import pandas as pd

# Global results directory (single source of truth)
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")

PHASE_NAME = {
    0: "stop",
    1: "square",
    2: "straight",
    3: "cw_rotation",
    4: "curve"
}


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_phase_segments(phase_csv: str, t_end_s: float) -> list:
    phases = pd.read_csv(phase_csv).sort_values("t_ns").reset_index(drop=True)

    if phases.empty:
        raise RuntimeError(f"Phase CSV '{phase_csv}' is empty")

    phase1 = phases[phases["phase"] == 1]

    if len(phase1) == 0:
        raise RuntimeError("Phase 1 not found in phase CSV. Cannot anchor segments.")

    t0_anchor_ns = int(phase1.iloc[0]["t_ns"])

    phases["t_s"] = (phases["t_ns"].astype(np.int64) - t0_anchor_ns) * 1e-9
    phases = phases.sort_values("t_s").reset_index(drop=True)

    phases = phases.drop_duplicates(subset=["phase"], keep="first").reset_index(drop=True)

    segs = []

    for i in range(len(phases)):
        ph = int(phases.loc[i, "phase"])
        t0 = float(phases.loc[i, "t_s"])

        if i + 1 < len(phases):
            t1 = float(phases.loc[i + 1, "t_s"])
        else:
            t1 = float(t_end_s)

        t0c = max(0.0, min(t0, t_end_s))
        t1c = max(0.0, min(t1, t_end_s))

        if t1c > t0c:
            segs.append((ph, t0c, t1c))

    return segs


def compute_segment_metrics(df: pd.DataFrame) -> dict:
    if df.empty:
        return {
            "n": 0,
            "ate_rmse_m": float("nan"),
            "ate_mean_m": float("nan"),
            "ate_max_m": float("nan"),
            "yaw_abs_mean_rad": float("nan"),
            "yaw_abs_max_rad": float("nan"),
        }

    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    ate = np.sqrt(ex * ex + ey * ey)

    yaw_err = df["gt_yaw"].values - df["est_yaw_al"].values
    yaw_err = np.array([wrap_pi(a) for a in yaw_err])
    yaw_abs = np.abs(yaw_err)

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
    df = pd.read_csv(aligned_csv)

    rows = []

    for ph, t0, t1 in segs:

        if include_phases is not None and ph not in include_phases:
            continue

        seg = df[(df["t_s"] >= t0) & (df["t_s"] < t1)]

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

    return pd.DataFrame(rows)


if __name__ == "__main__":

    import argparse

    ap = argparse.ArgumentParser()

    # Existing estimators
    ap.add_argument("--wheel", default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv"))
    ap.add_argument("--ekf", default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv"))
    ap.add_argument("--orb", default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv"))

    # ✅ NEW: cuVSLAM
    ap.add_argument("--cuvslam", default=os.path.join(RESULTS_DIR, "cuvslam_synced_aligned.csv"))

    ap.add_argument("--phase_csv", default=os.path.join(RESULTS_DIR, "gt_traj_phase_events.csv"))
    ap.add_argument("--out", default=os.path.join(RESULTS_DIR, "phase_metrics.csv"))

    ap.add_argument("--include", default="1,2,3,4")

    args = ap.parse_args()

    include_set = set(int(x.strip()) for x in args.include.split(",") if x.strip())

    # Load all trajectories
    wheel_df = pd.read_csv(args.wheel)
    ekf_df = pd.read_csv(args.ekf)
    orb_df = pd.read_csv(args.orb)
    cuvslam_df = pd.read_csv(args.cuvslam)

    # ✅ Include cuVSLAM in common time
    t_end = min(
        float(wheel_df["t_s"].max()),
        float(ekf_df["t_s"].max()),
        float(orb_df["t_s"].max()),
        float(cuvslam_df["t_s"].max())
    )

    print(f"Common trajectory end time: {t_end:.3f}s")

    segs = load_phase_segments(args.phase_csv, t_end_s=t_end)

    # Evaluate all estimators
    out_w = evaluate_by_phase(args.wheel, segs, "wheel", include_set)
    out_e = evaluate_by_phase(args.ekf, segs, "ekf", include_set)
    out_o = evaluate_by_phase(args.orb, segs, "orb", include_set)
    out_c = evaluate_by_phase(args.cuvslam, segs, "cuvslam", include_set)  # ✅ NEW

    # Combine
    out = pd.concat([out_w, out_e, out_o, out_c], ignore_index=True)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    out.to_csv(args.out, index=False)

    out = out.sort_values(["phase", "estimator"])

    print("\n" + out.to_string(index=False))
    print(f"\nSaved: {args.out}")
