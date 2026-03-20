import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ========================= GLOBAL RESULTS DIR =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")
# =====================================================================

PHASE_NAMES = {
    0: "STOP",
    1: "SQUARE",
    2: "STRAIGHT",
    3: "SPIN_CW",
    4: "CURVE"
}

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def unwrap_angle_series(a):
    return np.unwrap(np.asarray(a, dtype=float))


def compute_yaw_error(est_yaw, gt_yaw):
    return wrap_pi(est_yaw - gt_yaw)


def compute_yaw_rate(t, yaw_unwrapped):
    if len(t) < 2:
        return np.zeros_like(t)
    return np.gradient(yaw_unwrapped, t)


def summary_stats(name, yaw_err_rad):

    yaw_err_deg = np.rad2deg(yaw_err_rad)

    return {
        "estimator": name,
        "mean_abs_yaw_error_deg": float(np.mean(np.abs(yaw_err_deg))),
        "rmse_yaw_error_deg": float(np.sqrt(np.mean(yaw_err_deg ** 2))),
        "max_abs_yaw_error_deg": float(np.max(np.abs(yaw_err_deg))),
    }


def load_phase_segments(phase_csv, t_end):

    phases = pd.read_csv(phase_csv).sort_values("t_ns")

    t0_anchor = phases["t_ns"].iloc[0]
    phases["t_s"] = (phases["t_ns"] - t0_anchor) * 1e-9

    segs = []

    for i in range(len(phases)):

        ph = int(phases.iloc[i]["phase"])
        t0 = float(phases.iloc[i]["t_s"])

        if i + 1 < len(phases):
            t1 = float(phases.iloc[i + 1]["t_s"])
        else:
            t1 = t_end

        if t1 > 0:
            segs.append({
                "phase": ph,
                "name": PHASE_NAMES.get(ph, f"PHASE_{ph}"),
                "t_start": max(0, t0),
                "t_end": min(t1, t_end),
                "t_mid": 0.5 * (t0 + t1)
            })

    return segs


def add_phase_markings(ax, segs):

    if not segs:
        return

    ymin, ymax = ax.get_ylim()
    y_text = ymax - 0.05 * (ymax - ymin)

    for seg in segs:

        ax.axvline(
            seg["t_start"],
            color="black",
            linestyle="--",
            linewidth=1.2,
            alpha=0.6
        )

        ax.text(
            seg["t_mid"],
            y_text,
            seg["name"],
            ha="center",
            va="top",
            fontsize=9,
            bbox=dict(
                boxstyle="round,pad=0.2",
                facecolor="white",
                alpha=0.7,
                edgecolor="none"
            )
        )


if __name__ == "__main__":

    import argparse

    ap = argparse.ArgumentParser()

    # ========================= INPUTS =========================
    ap.add_argument("--wheel",
        default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv"))

    ap.add_argument("--ekf",
        default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv"))

    ap.add_argument("--orb",
        default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv"))  # ✅ ADDED

    ap.add_argument("--phase_csv",
        default=os.path.join(RESULTS_DIR, "gt_traj_phase_events.csv"))

    ap.add_argument("--out_dir",
        default=RESULTS_DIR)
    # =========================================================

    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Load
    wheel = pd.read_csv(args.wheel)
    ekf   = pd.read_csv(args.ekf)
    orb   = pd.read_csv(args.orb)   # ✅ ADDED

    # -------------------------------------------------
    # Synchronize time horizon
    # -------------------------------------------------

    t_end = min(
        float(wheel["t_s"].max()),
        float(ekf["t_s"].max()),
        float(orb["t_s"].max())   # ✅ ADDED
    )

    wheel = wheel[wheel["t_s"] <= t_end].reset_index(drop=True)
    ekf   = ekf[ekf["t_s"] <= t_end].reset_index(drop=True)
    orb   = orb[orb["t_s"] <= t_end].reset_index(drop=True)

    t = wheel["t_s"].values

    gt_yaw    = wheel["gt_yaw"].values
    wheel_yaw = wheel["est_yaw_al"].values
    ekf_yaw   = ekf["est_yaw_al"].values
    orb_yaw   = orb["est_yaw_al"].values   # ✅ ADDED

    # -------------------------------------------------
    # Errors
    # -------------------------------------------------

    wheel_err = compute_yaw_error(wheel_yaw, gt_yaw)
    ekf_err   = compute_yaw_error(ekf_yaw, gt_yaw)
    orb_err   = compute_yaw_error(orb_yaw, gt_yaw)   # ✅ ADDED

    # -------------------------------------------------
    # Unwrap
    # -------------------------------------------------

    gt_unw    = unwrap_angle_series(gt_yaw)
    wheel_unw = unwrap_angle_series(wheel_yaw)
    ekf_unw   = unwrap_angle_series(ekf_yaw)
    orb_unw   = unwrap_angle_series(orb_yaw)   # ✅ ADDED

    # -------------------------------------------------
    # Yaw rates
    # -------------------------------------------------

    gt_rate    = compute_yaw_rate(t, gt_unw)
    wheel_rate = compute_yaw_rate(t, wheel_unw)
    ekf_rate   = compute_yaw_rate(t, ekf_unw)
    orb_rate   = compute_yaw_rate(t, orb_unw)   # ✅ ADDED

    # -------------------------------------------------
    # Phase segments
    # -------------------------------------------------

    segs = load_phase_segments(args.phase_csv, t_end)

    # -------------------------------------------------
    # Plot 1: Heading comparison
    # -------------------------------------------------

    fig, ax = plt.subplots(figsize=(12,5))

    ax.plot(t, np.rad2deg(gt_yaw), color="black", label="GT", linewidth=2)
    ax.plot(t, np.rad2deg(wheel_yaw), label="Wheel", linewidth=1.5)
    ax.plot(t, np.rad2deg(ekf_yaw), label="EKF", linewidth=1.5)
    ax.plot(t, np.rad2deg(orb_yaw), label="ORB", linewidth=1.5)  # ✅ ADDED

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Yaw [deg]")
    ax.set_title("Heading Comparison")

    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()

    add_phase_markings(ax, segs)

    plt.tight_layout()
    plt.savefig(os.path.join(args.out_dir,"heading_comparison.png"), dpi=300)

    # -------------------------------------------------
    # Plot 2: Heading error
    # -------------------------------------------------

    fig, ax = plt.subplots(figsize=(12,5))

    ax.plot(t, np.rad2deg(wheel_err), label="Wheel Error", linewidth=1.5)
    ax.plot(t, np.rad2deg(ekf_err), label="EKF Error", linewidth=1.5)
    ax.plot(t, np.rad2deg(orb_err), label="ORB Error", linewidth=1.5)  # ✅ ADDED

    ax.axhline(0,color="black",linestyle="--")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Yaw Error [deg]")
    ax.set_title("Heading Error vs Time")

    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()

    add_phase_markings(ax, segs)

    plt.tight_layout()
    plt.savefig(os.path.join(args.out_dir,"heading_error.png"), dpi=300)

    # -------------------------------------------------
    # Plot 3: Heading rate
    # -------------------------------------------------

    fig, ax = plt.subplots(figsize=(12,5))

    ax.plot(t, np.rad2deg(gt_rate), color="black", label="GT", linewidth=2)
    ax.plot(t, np.rad2deg(wheel_rate), label="Wheel", linewidth=1.5)
    ax.plot(t, np.rad2deg(ekf_rate), label="EKF", linewidth=1.5)
    ax.plot(t, np.rad2deg(orb_rate), label="ORB", linewidth=1.5)  # ✅ ADDED

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Yaw Rate [deg/s]")
    ax.set_title("Heading Rate Comparison")

    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()

    add_phase_markings(ax, segs)

    plt.tight_layout()
    plt.savefig(os.path.join(args.out_dir,"heading_rate.png"), dpi=300)

    # -------------------------------------------------
    # Summary CSV
    # -------------------------------------------------

    summary = pd.DataFrame([
        summary_stats("Wheel", wheel_err),
        summary_stats("EKF", ekf_err),
        summary_stats("ORB", orb_err)   # ✅ ADDED
    ])

    summary.to_csv(
        os.path.join(args.out_dir,"heading_summary.csv"),
        index=False
    )

    print("Saved heading analysis outputs")

    plt.show()
