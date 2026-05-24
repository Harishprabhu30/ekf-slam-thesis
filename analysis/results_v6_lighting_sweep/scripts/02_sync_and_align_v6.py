import os
import json
import math
import argparse
import numpy as np
import pandas as pd

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

def trim_frozen_tail(df, eps=1e-6):
    if len(df) < 3:
        return df

    dx = np.diff(df["est_x"].values)
    dy = np.diff(df["est_y"].values)
    d = np.sqrt(dx * dx + dy * dy)

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
    if traj_df.empty:
        raise RuntimeError("Cannot compute motion t0 from empty trajectory")

    t = traj_df["t_ns"].values.astype(np.int64)
    x = traj_df["x"].values
    y = traj_df["y"].values

    if len(t) < 3:
        return int(t[0])

    dt = np.diff(t) * 1e-9
    dx = np.diff(x)
    dy = np.diff(y)

    dt = np.maximum(dt, 1e-6)
    speed = np.sqrt(dx * dx + dy * dy) / dt

    mean_dt = float(np.median(dt)) if len(dt) else 0.05
    k = max(3, int(sustain_s / max(mean_dt, 1e-3)))

    if len(speed) < k:
        return int(t[0])

    ma = np.convolve(speed, np.ones(k) / k, mode="same")

    idx = int(np.argmax(ma > vel_thresh))

    if ma[idx] <= vel_thresh:
        return int(t[0])

    return int(t[idx])

def sync_est_to_gt(gt_df, est_df, gt_t0_ns, est_t0_ns):
    gt_t = (gt_df["t_ns"].values.astype(np.int64) - int(gt_t0_ns)) * 1e-9
    est_t = (est_df["t_ns"].values.astype(np.int64) - int(est_t0_ns)) * 1e-9

    gt_mask = gt_t >= 0.0
    est_mask = est_t >= 0.0

    gt_t = gt_t[gt_mask]
    gt_df = gt_df.loc[gt_mask].reset_index(drop=True)

    est_t = est_t[est_mask]
    est_df = est_df.loc[est_mask].reset_index(drop=True)

    if len(gt_t) < 3:
        raise RuntimeError("Too few GT samples after t0 trim")

    if len(est_t) < 3:
        raise RuntimeError("Too few estimator samples after t0 trim")

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
    est_xy = np.vstack([
        df_sync["est_x"].values,
        df_sync["est_y"].values
    ])

    est_xy_al = (R @ est_xy).T + t

    rot = math.atan2(R[1, 0], R[0, 0])

    df = df_sync.copy()
    df["est_x_al"] = est_xy_al[:, 0]
    df["est_y_al"] = est_xy_al[:, 1]
    df["est_yaw_al"] = df["est_yaw"].apply(lambda a: wrap_pi(a + rot))

    return df

def path_len_xy(x, y):
    dx = np.diff(x)
    dy = np.diff(y)
    return float(np.sum(np.sqrt(dx * dx + dy * dy)))

def sync_and_align(
    gt_csv,
    est_csv,
    out_csv,
    meta_json=None,
    vel_thresh=0.02,
    sustain_s=0.5,
    align_start_s=2.0,
    align_dur_s=30.0,
):
    gt = pd.read_csv(gt_csv)
    est = pd.read_csv(est_csv)

    if gt.empty:
        raise RuntimeError(f"GT CSV is empty: {gt_csv}")

    if est.empty:
        raise RuntimeError(f"Estimator CSV is empty: {est_csv}")

    gt_t0 = compute_motion_t0(gt, vel_thresh, sustain_s)
    est_t0 = compute_motion_t0(est, vel_thresh, sustain_s)

    sync = sync_est_to_gt(gt, est, gt_t0, est_t0)
    sync = trim_frozen_tail(sync, eps=1e-6)

    if len(sync) < 50:
        raise RuntimeError(f"Too few synchronized samples: {len(sync)}")

    t0 = align_start_s
    t1 = t0 + align_dur_s

    win = sync[(sync["t_s"] >= t0) & (sync["t_s"] <= t1)]

    if len(win) < 50:
        win = sync

    R, t = se2_align_umeyama(
        win[["gt_x", "gt_y"]].values,
        win[["est_x", "est_y"]].values
    )

    aligned = apply_align(sync, R, t)

    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    aligned.to_csv(out_csv, index=False)

    L_gt = path_len_xy(sync["gt_x"].values, sync["gt_y"].values)
    L_est = path_len_xy(sync["est_x"].values, sync["est_y"].values)

    meta = {
        "gt_csv": gt_csv,
        "est_csv": est_csv,
        "out_csv": out_csv,
        "gt_t0_ns": int(gt_t0),
        "est_t0_ns": int(est_t0),
        "aligned_rows": int(len(aligned)),
        "alignment_window_rows": int(len(win)),
        "align_start_s": float(align_start_s),
        "align_dur_s": float(align_dur_s),
        "gt_path_length_before_alignment_m": float(L_gt),
        "est_path_length_before_alignment_m": float(L_est),
        "est_to_gt_path_ratio_before_alignment": float(L_est / L_gt) if L_gt > 1e-9 else float("nan"),
        "se2_rotation_rad": float(math.atan2(R[1, 0], R[0, 0])),
        "se2_translation_x_m": float(t[0]),
        "se2_translation_y_m": float(t[1]),
        "scale_applied": False,
    }

    if meta_json:
        os.makedirs(os.path.dirname(meta_json), exist_ok=True)
        with open(meta_json, "w") as f:
            json.dump(meta, f, indent=2)

    print(f"[sync] saved={out_csv}")
    print(f"[sync] rows={len(aligned)} | GT length={L_gt:.3f} m | EST length={L_est:.3f} m")
    print(f"[sync] motion t0 gt={gt_t0} est={est_t0}")
    print("[sync] scale_applied=False")

    return aligned, meta

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--meta_json", default=None)
    ap.add_argument("--vel_thresh", type=float, default=0.02)
    ap.add_argument("--sustain_s", type=float, default=0.5)
    ap.add_argument("--align_start_s", type=float, default=2.0)
    ap.add_argument("--align_dur_s", type=float, default=30.0)
    args = ap.parse_args()

    sync_and_align(
        gt_csv=args.gt,
        est_csv=args.est,
        out_csv=args.out,
        meta_json=args.meta_json,
        vel_thresh=args.vel_thresh,
        sustain_s=args.sustain_s,
        align_start_s=args.align_start_s,
        align_dur_s=args.align_dur_s,
    )

if __name__ == "__main__":
    main()
