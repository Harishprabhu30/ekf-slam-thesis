import numpy as np
import pandas as pd
import math
import os

# Global results directory (single source of truth)
RESULTS_DIR = os.getenv("TRAJ_RESULTS_DIR", "analysis/results_gt_traj_v6_orb/lowlight")


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def se2_align_umeyama(gt_xy: np.ndarray, est_xy: np.ndarray):
    mu_g = gt_xy.mean(axis=0)
    mu_e = est_xy.mean(axis=0)
    X = est_xy - mu_e
    Y = gt_xy - mu_g

    C = X.T @ Y / gt_xy.shape[0]
    U, _, Vt = np.linalg.svd(C)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = mu_g - R @ mu_e
    return R, t


def trim_frozen_tail(df, eps=1e-5):
    dx = np.diff(df["est_x"].values)
    dy = np.diff(df["est_y"].values)
    d = np.sqrt(dx*dx + dy*dy)
    idx = np.where(d > eps)[0]
    if len(idx) == 0:
        return df
    last = int(idx[-1] + 1)
    return df.iloc[: last + 1].reset_index(drop=True)


def interp_series(t_src, v_src, t_tgt):
    return np.interp(t_tgt, t_src, v_src)


def interp_yaw(t_src, yaw_src, t_tgt):
    y = np.unwrap(yaw_src)
    yi = np.interp(t_tgt, t_src, y)
    return np.array([wrap_pi(a) for a in yi])


def compute_motion_t0(traj_df: pd.DataFrame, vel_thresh=0.02, sustain_s=0.5):
    t = traj_df["t_ns"].values.astype(np.int64)
    x = traj_df["x"].values
    y = traj_df["y"].values

    dt = np.diff(t) * 1e-9
    dx = np.diff(x)
    dy = np.diff(y)
    dt = np.maximum(dt, 1e-6)
    speed = np.sqrt(dx*dx + dy*dy) / dt

    mean_dt = float(np.median(dt)) if len(dt) else 0.05
    k = max(3, int(sustain_s / mean_dt))

    if len(speed) < k:
        return int(t[0])
    ma = np.convolve(speed, np.ones(k)/k, mode="same")

    idx = np.argmax(ma > vel_thresh)
    if ma[idx] <= vel_thresh:
        return int(t[0])
    return int(t[idx])


def sync_est_to_gt(gt_df, est_df, gt_t0_ns, est_t0_ns):
    gt_t = (gt_df["t_ns"].values - gt_t0_ns) * 1e-9
    est_t = (est_df["t_ns"].values - est_t0_ns) * 1e-9

    gt_mask = gt_t >= 0.0
    est_mask = est_t >= 0.0
    gt_t = gt_t[gt_mask]
    gt_df = gt_df.loc[gt_mask].reset_index(drop=True)
    est_t = est_t[est_mask]
    est_df = est_df.loc[est_mask].reset_index(drop=True)

    t_end = min(float(gt_t.max()), float(est_t.max()))
    gt_keep = gt_t <= t_end
    gt_t = gt_t[gt_keep]
    gt_df = gt_df.loc[gt_keep].reset_index(drop=True)

    est_xi = interp_series(est_t, est_df["x"].values, gt_t)
    est_yi = interp_series(est_t, est_df["y"].values, gt_t)
    est_yawi = interp_yaw(est_t, est_df["yaw"].values, gt_t)

    out = pd.DataFrame({
        "t_s": gt_t,
        "gt_x": gt_df["x"].values,
        "gt_y": gt_df["y"].values,
        "gt_yaw": gt_df["yaw"].values,
        "est_x": est_xi,
        "est_y": est_yi,
        "est_yaw": est_yawi,
    })
    return out


def apply_align(df_sync, R, t):
    est_xy = np.vstack([df_sync["est_x"].values, df_sync["est_y"].values])
    est_xy_al = (R @ est_xy).T + t

    rot = math.atan2(R[1,0], R[0,0])

    df = df_sync.copy()
    df["est_x_al"] = est_xy_al[:, 0]
    df["est_y_al"] = est_xy_al[:, 1]
    df["est_yaw_al"] = df["est_yaw"].apply(lambda a: wrap_pi(a + rot))
    return df


# ========================= SCALE DIAGNOSTIC =========================
def path_len_xy(x, y):
    dx = np.diff(x)
    dy = np.diff(y)
    return float(np.sum(np.sqrt(dx*dx + dy*dy)))
# ====================================================================


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", default=os.path.join(RESULTS_DIR, "gt_traj_lowlight.csv"))
    ap.add_argument("--est", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--vel_thresh", type=float, default=0.02)
    ap.add_argument("--sustain_s", type=float, default=0.5)
    ap.add_argument("--align_start_s", type=float, default=2.0)
    ap.add_argument("--align_dur_s", type=float, default=30.0)

    # NEW ARGUMENT
    ap.add_argument("--diag_scale_est", action="store_true",
                    help="Diagnostic only: scale estimator trajectory by path-length ratio before alignment.")

    args = ap.parse_args()

    gt = pd.read_csv(args.gt)
    est = pd.read_csv(args.est)

    gt_t0 = compute_motion_t0(gt, args.vel_thresh, args.sustain_s)
    est_t0 = compute_motion_t0(est, args.vel_thresh, args.sustain_s)

    sync = sync_est_to_gt(gt, est, gt_t0, est_t0)

    # ========================= SCALE DIAGNOSTIC =========================
    if "orb" in args.est.lower():
        L_gt = path_len_xy(sync["gt_x"].values, sync["gt_y"].values)
        L_orb = path_len_xy(sync["est_x"].values, sync["est_y"].values)

        if L_orb > 1e-6:
            s = L_gt / L_orb
            print("\n[SCALE DIAGNOSTIC]")
            print(f"L_gt  = {L_gt:.3f} m")
            print(f"L_orb = {L_orb:.3f} m")
            print(f"scale s = L_gt / L_orb = {s:.3f}")
        else:
            print("\n[SCALE DIAGNOSTIC] ORB path too small")
    # ===================================================================

    # ========================= DIAGNOSTIC SCALE APPLY =========================
    if args.diag_scale_est and "orb" in args.est.lower():
        L_gt = path_len_xy(sync["gt_x"].values, sync["gt_y"].values)
        L_est = path_len_xy(sync["est_x"].values, sync["est_y"].values)

        if L_est > 1e-6:
            s = L_gt / L_est
            sync["est_x"] = sync["est_x"] * s
            sync["est_y"] = sync["est_y"] * s
            print(f"\n[DIAGNOSTIC SCALE APPLIED] s = {s:.3f}")
        else:
            print("\n[DIAGNOSTIC SCALE APPLIED] skipped, estimator path too small")
    # =========================================================================

    # ---------------------------- TRIM FROZEN TAIL ----------------------------
    sync = trim_frozen_tail(sync, eps=1e-6)

    # ---------------------------- ALIGN ----------------------------
    t0 = args.align_start_s
    t1 = t0 + args.align_dur_s
    win = sync[(sync["t_s"] >= t0) & (sync["t_s"] <= t1)]
    if len(win) < 50:
        win = sync

    R, t = se2_align_umeyama(
        win[["gt_x","gt_y"]].values,
        win[["est_x","est_y"]].values
    )
    aligned = apply_align(sync, R, t)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    aligned.to_csv(args.out, index=False)

    print(f"\nSaved synced+aligned: {args.out}")
    print(f"Motion t0 (gt, est) ns: {gt_t0}, {est_t0}")
