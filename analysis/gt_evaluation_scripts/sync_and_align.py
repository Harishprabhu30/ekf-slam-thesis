import numpy as np
import pandas as pd
import math
import argparse, os

# ------------------- helpers -------------------

def load_phase_t0(phase_csv):
    df = pd.read_csv(phase_csv)
    # pick earliest phase==1
    df1 = df[df["phase"] == 1]
    if len(df1) == 0:
        # fallback: earliest event
        return int(df["t_ns"].iloc[0])
    return int(df1["t_ns"].iloc[0])

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def se2_align_umeyama(gt_xy: np.ndarray, est_xy: np.ndarray):
    """
    Find R,t minimizing ||gt - (R*est + t)|| in least squares (no scale).
    Returns R (2x2), t (2,)
    """
    assert gt_xy.shape == est_xy.shape
    mu_g = gt_xy.mean(axis=0)
    mu_e = est_xy.mean(axis=0)
    X = est_xy - mu_e
    Y = gt_xy - mu_g

    C = X.T @ Y / gt_xy.shape[0]
    U, _, Vt = np.linalg.svd(C)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:  # fix reflection
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = mu_g - R @ mu_e
    return R, t

def interp_series(t_src, v_src, t_tgt):
    return np.interp(t_tgt, t_src, v_src)

def interp_yaw(t_src, yaw_src, t_tgt):
    y = np.unwrap(yaw_src)
    yi = np.interp(t_tgt, t_src, y)
    return np.array([wrap_pi(a) for a in yi])

def sync_est_to_gt(gt_df, est_df, gt_t0_ns, est_t0_ns):
    """
    Interpolate estimator trajectory onto GT times, using provided start times
    """
    gt_t = (gt_df["t_ns"].values - gt_t0_ns) * 1e-9
    est_t = (est_df["t_ns"].values - est_t0_ns) * 1e-9

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

    # Restrict to common valid time window
    t_end = min(gt_t.max(), est_t.max())
    out = out[(out["t_s"] >= 0) & (out["t_s"] <= t_end)].reset_index(drop=True)

    return out

def apply_align(df_sync, R, t):
    est_xy = np.vstack([df_sync["est_x"].values, df_sync["est_y"].values])
    est_xy_al = (R @ est_xy).T + t
    df = df_sync.copy()
    df["est_x_al"] = est_xy_al[:, 0]
    df["est_y_al"] = est_xy_al[:, 1]

    rot = math.atan2(R[1,0], R[0,0])
    df["est_yaw_al"] = df["est_yaw"].apply(lambda a: wrap_pi(a + rot))
    return df

# ------------------- main -------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", default="analysis/results_gt/gt_traj.csv")
    ap.add_argument("--gt_phase", help="CSV with GT phase events")
    ap.add_argument("--est", required=True)
    ap.add_argument("--est_phase", help="CSV with estimator phase events")
    ap.add_argument("--out", required=True)
    ap.add_argument("--skip", type=int, default=50, help="Skip first N samples to avoid initial jitter")
    args = ap.parse_args()

    gt = pd.read_csv(args.gt)
    est = pd.read_csv(args.est)

    # Determine t0 based on phase events if provided
    gt_t0 = load_phase_t0(args.gt_phase) if args.gt_phase else gt["t_ns"].iloc[0]
    est_t0 = load_phase_t0(args.est_phase) if args.est_phase else est["t_ns"].iloc[0]

    # Sync trajectories
    sync = sync_est_to_gt(gt, est, gt_t0_ns=gt_t0, est_t0_ns=est_t0)

    # Align using XY after skipping first N samples
    s = max(0, args.skip)
    gt_xy = sync[["gt_x", "gt_y"]].values[s:]
    est_xy = sync[["est_x", "est_y"]].values[s:]

    R, t = se2_align_umeyama(gt_xy, est_xy)
    aligned = apply_align(sync, R, t)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    aligned.to_csv(args.out, index=False)
    print(f"Saved synced+aligned: {args.out}")
