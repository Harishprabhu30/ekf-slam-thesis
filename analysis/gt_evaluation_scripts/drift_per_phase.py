import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ========================= GLOBAL RESULTS DIR =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")
# =====================================================================


PHASE_NAME = {
    0: "stop",
    1: "square",
    2: "straight",
    3: "cw_rotation",
    4: "circular"
}


def load_phase_boundaries_phase1_anchor(phase_csv: str) -> pd.DataFrame:
    """
    Convert phase event record-time (t_ns) to aligned seconds t_s using phase==1 as t0.
    """
    phases = pd.read_csv(phase_csv).sort_values("t_ns").reset_index(drop=True)

    if phases.empty:
        raise RuntimeError(f"Phase CSV '{phase_csv}' is empty")

    phase1 = phases[phases["phase"] == 1]
    if len(phase1) == 0:
        raise RuntimeError("Phase 1 not found in phase CSV.")

    t0_anchor_ns = int(phase1.iloc[0]["t_ns"])
    phases["t_s"] = (phases["t_ns"].astype(np.int64) - t0_anchor_ns) * 1e-9

    phases = phases.sort_values("t_s").reset_index(drop=True)
    phases = phases.drop_duplicates(subset=["phase"], keep="first").reset_index(drop=True)

    return phases[["t_s", "phase"]]


def compute_distance(df: pd.DataFrame) -> float:
    x = df["gt_x"].values
    y = df["gt_y"].values
    dx = np.diff(x)
    dy = np.diff(y)
    return np.sum(np.sqrt(dx*dx + dy*dy))


def ate_series(df: pd.DataFrame) -> np.ndarray:
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    return np.sqrt(ex*ex + ey*ey)


def compute_phase_metrics(df: pd.DataFrame, phases: pd.DataFrame):

    results = []

    for i in range(len(phases) - 1):

        t_start = float(phases.iloc[i]["t_s"])
        t_stop = float(phases.iloc[i + 1]["t_s"])
        phase_id = int(phases.iloc[i]["phase"])

        seg = df[(df["t_s"] >= t_start) & (df["t_s"] < t_stop)]

        if len(seg) < 2:
            continue

        length = compute_distance(seg)

        ate = ate_series(seg)
        final_ate = float(ate[-1])

        # Avoid unstable drift for very short segments
        if length < 0.2:
            drift_rate = np.nan
        else:
            drift_rate = final_ate / length

        results.append({
            "phase": phase_id,
            "phase_name": PHASE_NAME.get(phase_id, "?"),
            "phase_length_m": length,
            "final_ate_m": final_ate,
            "drift_rate_m_per_m": drift_rate
        })

    return pd.DataFrame(results)


if __name__ == "__main__":

    import argparse

    ap = argparse.ArgumentParser()

    # ========================= INPUTS =========================
    ap.add_argument("--wheel", default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv"))
    ap.add_argument("--ekf",   default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv"))
    ap.add_argument("--orb",   default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv"))  # ADDED

    ap.add_argument("--phase_csv", default=os.path.join(RESULTS_DIR, "gt_traj_phase_events.csv"))

    ap.add_argument("--out_csv",  default=os.path.join(RESULTS_DIR, "drift_per_phase.csv"))
    ap.add_argument("--out_plot", default=os.path.join(RESULTS_DIR, "drift_per_phase_plot.png"))
    # =========================================================

    args = ap.parse_args()

    # Load data
    wheel = pd.read_csv(args.wheel)
    ekf   = pd.read_csv(args.ekf)
    orb   = pd.read_csv(args.orb)   # ✅ ADDED

    phases = load_phase_boundaries_phase1_anchor(args.phase_csv)

    # ========================= COMMON TIME HORIZON =========================
    t_end = min(
        float(wheel["t_s"].max()),
        float(ekf["t_s"].max()),
        float(orb["t_s"].max())   # ADDED
    )

    wheel = wheel[wheel["t_s"] <= t_end].reset_index(drop=True)
    ekf   = ekf[ekf["t_s"] <= t_end].reset_index(drop=True)
    orb   = orb[orb["t_s"] <= t_end].reset_index(drop=True)
    # ======================================================================

    # ========================= PER-PHASE METRICS =========================
    wheel_phase = compute_phase_metrics(wheel, phases)
    wheel_phase["estimator"] = "wheel"

    ekf_phase = compute_phase_metrics(ekf, phases)
    ekf_phase["estimator"] = "ekf"

    orb_phase = compute_phase_metrics(orb, phases)   # ADDED
    orb_phase["estimator"] = "orb"
    # =====================================================================

    df_out = pd.concat([wheel_phase, ekf_phase, orb_phase], ignore_index=True)

    print("\nDrift per Phase\n")
    print(df_out)

    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    df_out.to_csv(args.out_csv, index=False)

    print(f"\nSaved CSV: {args.out_csv}")

    # ========================= PLOT =========================
    wheel_df = df_out[df_out["estimator"] == "wheel"]
    ekf_df   = df_out[df_out["estimator"] == "ekf"]
    orb_df   = df_out[df_out["estimator"] == "orb"]   # ADDED

    phases_ids = wheel_df["phase"].values
    phase_labels = wheel_df["phase_name"].values

    wheel_drift = wheel_df["drift_rate_m_per_m"].values
    ekf_drift   = ekf_df["drift_rate_m_per_m"].values
    orb_drift   = orb_df["drift_rate_m_per_m"].values   # ADDED

    x = np.arange(len(phases_ids))
    width = 0.25   # narrower bars for 3 estimators

    plt.figure(figsize=(10,5))

    plt.bar(x - width, wheel_drift, width, label="Wheel")
    plt.bar(x,         ekf_drift,   width, label="EKF")
    plt.bar(x + width, orb_drift,   width, label="ORB")   # ADDED

    plt.xticks(x, phase_labels)
    plt.ylabel("Drift Rate [m/m]")
    plt.xlabel("Trajectory Phase")
    plt.title("Drift per Phase")
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.legend()

    os.makedirs(os.path.dirname(args.out_plot), exist_ok=True)

    plt.tight_layout()
    plt.savefig(args.out_plot, dpi=300)

    print(f"Saved Plot: {args.out_plot}")

    plt.show()
