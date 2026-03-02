import pandas as pd
import numpy as np

def safe_improve(w, e):
    if w == 0 or np.isnan(w) or np.isnan(e):
        return np.nan
    return (w - e) / w * 100.0

if __name__ == "__main__":
    df = pd.read_csv("analysis/results/metrics_phasewise.csv")

    wheel = df[df["estimator"]=="wheel"].set_index("phase")
    ekf   = df[df["estimator"]=="ekf"].set_index("phase")

    phases = [p for p in wheel.index if p in ekf.index]

    rows = []
    for ph in phases:
        w = wheel.loc[ph]
        e = ekf.loc[ph]

        rows.append({
            "phase": ph,
            "drift_improve_%": safe_improve(w["drift_m"], e["drift_m"]),
            "omega_var_improve_%": safe_improve(w["omega_var"], e["omega_var"]),
            "jerk_var_improve_%": safe_improve(w["jerk_var"], e["jerk_var"]),
            "abs_delta_yaw_improve_%": safe_improve(abs(w["delta_yaw_rad"]), abs(e["delta_yaw_rad"])),
        })

    out = pd.DataFrame(rows)
    out.to_csv("analysis/results/improvement_table.csv", index=False)
    print("✅ Saved analysis/results/improvement_table.csv")
    print(out)
