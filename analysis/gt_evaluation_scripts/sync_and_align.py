import numpy as np
import pandas as pd
import math

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
    # Fix reflection
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = mu_g - R @ mu_e
    return R, t

def interp_series(t_src, v_src, t_tgt):
    return np.interp(t_tgt, t_src, v_src)

def interp_yaw(t_src, yaw_src, t_tgt):
    # unwrap -> interp -> wrap
    y = np.unwrap(yaw_src)
    yi = np.interp(t_tgt, t_src, y)
    return np.array([wrap_pi(a) for a in yi])

def sync_est_to_gt(gt_df, est_df):
    # Relative time in seconds
    gt_t0 = gt_df["t_ns"].iloc[0]
    est_t0 = est_df["t_ns"].iloc[0]
    gt_t = (gt_df["t_ns"].values - gt_t0) * 1e-9
    est_t = (est_df["t_ns"].values - est_t0) * 1e-9

    # Interpolate estimator onto GT times
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
    df = df_sync.copy()
    df["est_x_al"] = est_xy_al[:, 0]
    df["est_y_al"] = est_xy_al[:, 1]

    # yaw alignment: add rotation angle of R
    rot = math.atan2(R[1,0], R[0,0])
    df["est_yaw_al"] = df["est_yaw"].apply(lambda a: wrap_pi(a + rot))
    return df

if __name__ == "__main__":
    import argparse, os
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", default="analysis/results_gt/gt_traj.csv")
    ap.add_argument("--est", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--skip", type=int, default=50, help="Skip first N samples to avoid initial jitter")
    args = ap.parse_args()

    gt = pd.read_csv(args.gt)
    est = pd.read_csv(args.est)

    sync = sync_est_to_gt(gt, est)

    # Align using XY after skipping first N points
    s = max(0, args.skip)
    gt_xy = sync[["gt_x", "gt_y"]].values[s:]
    est_xy = sync[["est_x", "est_y"]].values[s:]

    R, t = se2_align_umeyama(gt_xy, est_xy)
    aligned = apply_align(sync, R, t)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    aligned.to_csv(args.out, index=False)
    print(f"Saved synced+aligned: {args.out}")
