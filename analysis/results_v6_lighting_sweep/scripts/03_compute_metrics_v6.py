import os
import math
import argparse
import numpy as np
import pandas as pd

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def rmse(x):
    return float(np.sqrt(np.mean(x * x))) if len(x) else float("nan")

def mean(x):
    return float(np.mean(x)) if len(x) else float("nan")

def mx(x):
    return float(np.max(x)) if len(x) else float("nan")

def path_len_xy(x, y):
    dx = np.diff(x)
    dy = np.diff(y)
    return float(np.sum(np.sqrt(dx * dx + dy * dy)))

def ate_errors(df):
    ex = df["gt_x"].values - df["est_x_al"].values
    ey = df["gt_y"].values - df["est_y_al"].values
    return np.sqrt(ex * ex + ey * ey)

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

    if len(t) < 3 or float(t.max()) < delta_s:
        return np.array([]), np.array([])

    t2 = t + delta_s

    gt_x2 = np.interp(t2, t, gt_x)
    gt_y2 = np.interp(t2, t, gt_y)
    gt_yaw2 = np.interp(t2, t, np.unwrap(gt_yaw))

    est_x2 = np.interp(t2, t, est_x)
    est_y2 = np.interp(t2, t, est_y)
    est_yaw2 = np.interp(t2, t, np.unwrap(est_yaw))

    def rel(dx, dy, dyaw, yaw0):
        c = np.cos(-yaw0)
        s = np.sin(-yaw0)

        rx = c * dx - s * dy
        ry = s * dx + c * dy

        return rx, ry, np.array([wrap_pi(a) for a in dyaw])

    gt_rx, gt_ry, gt_dyaw = rel(
        gt_x2 - gt_x,
        gt_y2 - gt_y,
        gt_yaw2 - gt_yaw,
        gt_yaw
    )

    est_rx, est_ry, est_dyaw = rel(
        est_x2 - est_x,
        est_y2 - est_y,
        est_yaw2 - est_yaw,
        est_yaw
    )

    rpe_trans = np.sqrt((gt_rx - est_rx) ** 2 + (gt_ry - est_ry) ** 2)
    rpe_yaw = np.abs(np.array([wrap_pi(a) for a in (gt_dyaw - est_dyaw)]))

    valid = t2 <= t.max()

    return rpe_trans[valid], rpe_yaw[valid]

def compute_metrics(
    aligned_csv,
    gt_csv,
    est_csv,
    lighting,
    trial,
    estimator,
    run_name,
    delta_s=1.0,
):
    df = pd.read_csv(aligned_csv)
    gt_df = pd.read_csv(gt_csv)
    est_df = pd.read_csv(est_csv)

    if df.empty:
        raise RuntimeError(f"Aligned CSV is empty: {aligned_csv}")

    ate = ate_errors(df)
    yaw_abs = yaw_errors(df)
    rpe_t, rpe_y = rpe_errors(df, delta_s=delta_s)

    gt_path_length_m = path_len_xy(df["gt_x"].values, df["gt_y"].values)
    est_path_length_aligned_m = path_len_xy(df["est_x_al"].values, df["est_y_al"].values)

    ate_rmse_m = rmse(ate)
    final_ate_m = float(ate[-1]) if len(ate) else float("nan")

    duration_s = float(df["t_s"].max() - df["t_s"].min()) if len(df) else float("nan")

    gt_rows = int(len(gt_df))
    est_rows = int(len(est_df))
    pose_count_ratio = float(est_rows / gt_rows) if gt_rows > 0 else float("nan")

    yaw_mean_rad = mean(yaw_abs)
    yaw_max_rad = mx(yaw_abs)

    rpe_y_rmse_rad = rmse(rpe_y)

    row = {
        "lighting": lighting,
        "trial": trial,
        "estimator": estimator,
        "run_name": run_name,

        "n_aligned": int(len(df)),
        "duration_s": duration_s,
        "gt_rows": gt_rows,
        "est_rows": est_rows,
        "pose_count_ratio": pose_count_ratio,

        "gt_path_length_m": gt_path_length_m,
        "est_path_length_aligned_m": est_path_length_aligned_m,

        "ate_rmse_m": ate_rmse_m,
        "ate_mean_m": mean(ate),
        "ate_max_m": mx(ate),
        "ate_final_m": final_ate_m,
        "ate_norm_rmse": float(ate_rmse_m / gt_path_length_m) if gt_path_length_m > 1e-9 else float("nan"),

        "drift_rate_m_per_m": float(final_ate_m / gt_path_length_m) if gt_path_length_m > 1e-9 else float("nan"),

        "yaw_abs_mean_rad": yaw_mean_rad,
        "yaw_abs_max_rad": yaw_max_rad,
        "yaw_abs_mean_deg": float(np.degrees(yaw_mean_rad)) if not math.isnan(yaw_mean_rad) else float("nan"),
        "yaw_abs_max_deg": float(np.degrees(yaw_max_rad)) if not math.isnan(yaw_max_rad) else float("nan"),

        f"rpe_trans_rmse_m_{delta_s:.1f}s": rmse(rpe_t),
        f"rpe_yaw_rmse_rad_{delta_s:.1f}s": rpe_y_rmse_rad,
        f"rpe_yaw_rmse_deg_{delta_s:.1f}s": float(np.degrees(rpe_y_rmse_rad)) if not math.isnan(rpe_y_rmse_rad) else float("nan"),

        "aligned_csv": aligned_csv,
        "gt_csv": gt_csv,
        "est_csv": est_csv,
    }

    return row

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--aligned_csv", required=True)
    ap.add_argument("--gt_csv", required=True)
    ap.add_argument("--est_csv", required=True)
    ap.add_argument("--lighting", required=True)
    ap.add_argument("--trial", required=True)
    ap.add_argument("--estimator", required=True)
    ap.add_argument("--run_name", required=True)
    ap.add_argument("--delta_s", type=float, default=1.0)
    ap.add_argument("--out_csv", required=True)
    args = ap.parse_args()

    row = compute_metrics(
        aligned_csv=args.aligned_csv,
        gt_csv=args.gt_csv,
        est_csv=args.est_csv,
        lighting=args.lighting,
        trial=args.trial,
        estimator=args.estimator,
        run_name=args.run_name,
        delta_s=args.delta_s,
    )

    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)

    if os.path.exists(args.out_csv):
        out = pd.read_csv(args.out_csv)
        out = out[out["run_name"] != args.run_name]
        out = pd.concat([out, pd.DataFrame([row])], ignore_index=True)
    else:
        out = pd.DataFrame([row])

    out.to_csv(args.out_csv, index=False)

    print(f"[metrics] saved/updated={args.out_csv}")
    print(pd.DataFrame([row])[[
        "lighting", "trial", "estimator",
        "ate_rmse_m", "ate_norm_rmse",
        "yaw_abs_mean_deg",
        "drift_rate_m_per_m",
        "pose_count_ratio"
    ]].to_string(index=False))

if __name__ == "__main__":
    main()
