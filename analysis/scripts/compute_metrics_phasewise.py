import pandas as pd
import numpy as np

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def block_metrics(df):
    df = df.dropna(subset=["x","y","yaw","omega_z","timestamp"]).copy()
    if len(df) < 10:
        return None

    t = df["timestamp"].to_numpy(dtype=np.float64) * 1e-9
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    yaw = df["yaw"].to_numpy()
    omega = df["omega_z"].to_numpy()

    drift = float(np.hypot(x[-1]-x[0], y[-1]-y[0]))
    dyaw = float(wrap_pi(yaw[-1]-yaw[0]))

    omega_var = float(np.nanvar(omega))

    dt = np.diff(t)
    dt[dt == 0] = np.nan
    jerk = np.diff(omega) / dt
    jerk_var = float(np.nanvar(jerk))

    return {
        "N": int(len(df)),
        "drift_m": drift,
        "delta_yaw_rad": dyaw,
        "omega_var": omega_var,
        "jerk_var": jerk_var
    }

def compute(csv_path, estimator_name):
    df = pd.read_csv(csv_path)

    df = df[df["phase"] != "unknown"]

    rows = []
    overall = block_metrics(df)
    if overall:
        rows.append({"estimator": estimator_name, "phase": "ALL", **overall})

    for ph, g in df.groupby("phase"):
        m = block_metrics(g)
        if m:
            rows.append({"estimator": estimator_name, "phase": str(ph), **m})

    return pd.DataFrame(rows)

if __name__ == "__main__":
    wheel = compute("analysis/results/wheel_labeled_final.csv", "wheel")
    ekf   = compute("analysis/results/ekf_labeled_final.csv", "ekf")

    out = pd.concat([wheel, ekf], ignore_index=True)
    out.to_csv("analysis/results/metrics_phasewise.csv", index=False)
    print("Saved analysis/results/metrics_phasewise.csv")
    print(out)
