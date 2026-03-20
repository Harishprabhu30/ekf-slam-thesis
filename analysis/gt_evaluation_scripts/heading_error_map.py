import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ========================= GLOBAL RESULTS DIR =========================
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v5_orb")
# =====================================================================

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def compute_yaw_error(est_yaw, gt_yaw):
    return wrap_pi(est_yaw - gt_yaw)


if __name__ == "__main__":

    import argparse

    ap = argparse.ArgumentParser()

    # ========================= INPUTS =========================
    ap.add_argument(
        "--wheel",
        default=os.path.join(RESULTS_DIR, "wheel_synced_aligned.csv")
    )

    ap.add_argument(
        "--ekf",
        default=os.path.join(RESULTS_DIR, "ekf_synced_aligned.csv")
    )

    ap.add_argument(
        "--orb",
        default=os.path.join(RESULTS_DIR, "orb_synced_aligned.csv")  # ✅ ADDED for ORB
    )

    ap.add_argument(
        "--out_dir",
        default=RESULTS_DIR
    )

    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # ========================= Load Data =========================
    wheel = pd.read_csv(args.wheel)
    ekf = pd.read_csv(args.ekf)
    orb = pd.read_csv(args.orb)  # ✅ ADDED for ORB

    # ----------------------------------
    # Match common time horizon
    # ----------------------------------

    t_end = min(
        wheel["t_s"].max(),
        ekf["t_s"].max(),
        orb["t_s"].max()   # ✅ ADDED for ORB
    )

    wheel = wheel[wheel["t_s"] <= t_end].reset_index(drop=True)
    ekf = ekf[ekf["t_s"] <= t_end].reset_index(drop=True)
    orb = orb[orb["t_s"] <= t_end].reset_index(drop=True)  # ✅ ADDED for ORB

    # Ground truth trajectory
    gt_x = wheel["gt_x"].values
    gt_y = wheel["gt_y"].values
    gt_yaw = wheel["gt_yaw"].values

    # Estimator yaw
    wheel_yaw = wheel["est_yaw_al"].values
    ekf_yaw = ekf["est_yaw_al"].values
    orb_yaw = orb["est_yaw_al"].values   # ✅ ADDED for ORB

    # ----------------------------------
    # Compute heading errors
    # ----------------------------------

    wheel_err = np.abs(np.rad2deg(compute_yaw_error(wheel_yaw, gt_yaw)))
    ekf_err = np.abs(np.rad2deg(compute_yaw_error(ekf_yaw, gt_yaw)))
    orb_err = np.abs(np.rad2deg(compute_yaw_error(orb_yaw, gt_yaw)))   # ✅ ADDED for ORB

    # ----------------------------------
    # Create figure
    # ----------------------------------

    fig, axs = plt.subplots(3, 1, figsize=(8, 18))  # Changed to 3 subplots

    # ------------------------------
    # Wheel heading error map
    # ------------------------------

    sc1 = axs[0].scatter(
        gt_x,
        gt_y,
        c=wheel_err,
        cmap="plasma",
        s=10
    )

    axs[0].plot(gt_x, gt_y, color="black", linewidth=0.7, alpha=0.4)

    axs[0].set_title("Wheel Odometry Heading Error Map")
    axs[0].set_xlabel("X [m]")
    axs[0].set_ylabel("Y [m]")
    axs[0].axis("equal")
    axs[0].grid(True, linestyle="--", alpha=0.4)

    plt.colorbar(sc1, ax=axs[0], label="|Heading Error| [deg]")

    # ------------------------------
    # EKF heading error map
    # ------------------------------

    sc2 = axs[1].scatter(
        gt_x,
        gt_y,
        c=ekf_err,
        cmap="viridis",
        s=10
    )

    axs[1].plot(gt_x, gt_y, color="black", linewidth=0.7, alpha=0.4)

    axs[1].set_title("EKF Heading Error Map")
    axs[1].set_xlabel("X [m]")
    axs[1].set_ylabel("Y [m]")
    axs[1].axis("equal")
    axs[1].grid(True, linestyle="--", alpha=0.4)

    plt.colorbar(sc2, ax=axs[1], label="|Heading Error| [deg]")

    # ------------------------------
    # ORB heading error map (new)
    # ------------------------------

    sc3 = axs[2].scatter(
        gt_x,
        gt_y,
        c=orb_err,
        cmap="inferno",   # Different color map for ORB
        s=10
    )

    axs[2].plot(gt_x, gt_y, color="black", linewidth=0.7, alpha=0.4)

    axs[2].set_title("ORB Odometry Heading Error Map")
    axs[2].set_xlabel("X [m]")
    axs[2].set_ylabel("Y [m]")
    axs[2].axis("equal")
    axs[2].grid(True, linestyle="--", alpha=0.4)

    plt.colorbar(sc3, ax=axs[2], label="|Heading Error| [deg]")

    # ----------------------------------
    # Save figure
    # ----------------------------------

    plt.tight_layout()

    out = os.path.join(args.out_dir, "heading_error_maps.png")

    plt.savefig(out, dpi=300)

    print("Saved:", out)

    plt.show()
