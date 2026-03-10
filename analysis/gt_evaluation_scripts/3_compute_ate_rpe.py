import numpy as np
import pandas as pd
import math

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def ate_errors(df):
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    e = np.sqrt(ex*ex + ey*ey)
    return e

def yaw_errors(df):
    dy = df["gt_yaw"].values - df["est_yaw_al"].values
    dy = np.array([wrap_pi(a) for a in dy])
    return np.abs(dy)

def rpe_errors(df, delta_s=1.0):
    t = df["t_s"].values
    gt_x = df["gt_x"].values
    gt_y = df["gt_y"].values
    gt_yaw = df["gt_yaw"].values

    est_x = df["est_x_al"].values
    est_y = df["est_y_al"].values
    est_yaw = df["est_yaw_al"].values

    # target times t+delta
    t2 = t + delta_s
    # interpolate indices by time
    gt_x2 = np.interp(t2, t, gt_x)
    gt_y2 = np.interp(t2, t, gt_y)
    gt_yaw2 = np.interp(t2, t, np.unwrap(gt_yaw))

    est_x2 = np.interp(t2, t, est_x)
    est_y2 = np.interp(t2, t, est_y)
    est_yaw2 = np.interp(t2, t, np.unwrap(est_yaw))

    # relative motion (SE2) from t to t+delta
    def rel(dx, dy, dyaw, yaw0):
        # express translation in frame at t (rotate by -yaw0)
        c = np.cos(-yaw0); s = np.sin(-yaw0)
        rx = c*dx - s*dy
        ry = s*dx + c*dy
        return rx, ry, wrap_pi(dyaw)

    gt_rx, gt_ry, gt_dyaw = rel(gt_x2-gt_x, gt_y2-gt_y, gt_yaw2-gt_yaw, gt_yaw)
    est_rx, est_ry, est_dyaw = rel(est_x2-est_x, est_y2-est_y, est_yaw2-est_yaw, est_yaw)

    # error between relative motions
    rpe_trans = np.sqrt((gt_rx-est_rx)**2 + (gt_ry-est_ry)**2)
    rpe_yaw = np.abs(np.array([wrap_pi(a) for a in (gt_dyaw - est_dyaw)]))

    # ignore last samples where t+delta exceeds range (interp will clamp -> bad)
    valid = t2 <= t.max()
    return rpe_trans[valid], rpe_yaw[valid]

def summarize(name, ate, yaw_abs, rpe_t, rpe_y, delta_s):
    def rmse(x): return float(np.sqrt(np.mean(x*x))) if len(x) else float("nan")
    def mean(x): return float(np.mean(x)) if len(x) else float("nan")
    def mx(x): return float(np.max(x)) if len(x) else float("nan")

    return {
        "name": name,
        "n": int(len(ate)),
        "ate_rmse_m": rmse(ate),
        "ate_mean_m": mean(ate),
        "ate_max_m": mx(ate),
        "yaw_abs_mean_rad": mean(yaw_abs),
        "yaw_abs_max_rad": mx(yaw_abs),
        f"rpe_trans_rmse_m@{delta_s}s": rmse(rpe_t),
        f"rpe_yaw_rmse_rad@{delta_s}s": rmse(rpe_y),
    }

if __name__ == "__main__":
    import argparse, os

    ap = argparse.ArgumentParser()
    ap.add_argument("--aligned_csv", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--delta_s", type=float, default=1.0)
    ap.add_argument("--out_csv", default="analysis/results_gt_traj_v3/metrics_vs_gt.csv")
    args = ap.parse_args()

    df = pd.read_csv(args.aligned_csv)

    ate = ate_errors(df)
    yaw_abs = yaw_errors(df)
    rpe_t, rpe_y = rpe_errors(df, delta_s=args.delta_s)

    row = summarize(args.name, ate, yaw_abs, rpe_t, rpe_y, args.delta_s)

    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)

    # append or create
    if os.path.exists(args.out_csv):
        out = pd.read_csv(args.out_csv)
        out = out[out["name"] != args.name]  # replace if exists
        out = pd.concat([out, pd.DataFrame([row])], ignore_index=True)
    else:
        out = pd.DataFrame([row])

    out.to_csv(args.out_csv, index=False)
    print(out.sort_values("name").to_string(index=False))
