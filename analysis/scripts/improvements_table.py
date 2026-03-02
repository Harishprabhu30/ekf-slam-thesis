(analysis_env) vgtu@vgtu-B450M-DS3H:~/Downloads/Harish_Thesis/ros2_ws/analysis/scripts$ pwd
ls -l improvement_table.py
python3 -c "import os; print(os.path.abspath('improvement_table.py'))"
/home/vgtu/Downloads/Harish_Thesis/ros2_ws/analysis/scripts
-rw-rw-r-- 1 vgtu vgtu 1087 Mar  2 15:22 improvement_table.py
/home/vgtu/Downloads/Harish_Thesis/ros2_ws/analysis/scripts/improvement_table.py
(analysis_env) vgtu@vgtu-B450M-DS3H:~/Downloads/Harish_Thesis/ros2_ws/analysis/scripts$ python3 improvement_table.py
✅ Saved analysis/results/improvement_table.csv
  phase  drift_improve_%  ...  jerk_var_improve_%  abs_delta_yaw_improve_%
0   ALL        -0.126749  ...           93.784639                 3.935487

[1 rows x 5 columns]
(analysis_env) vgtu@vgtu-B450M-DS3H:~/Downloads/Harish_Thesis/ros2_ws/analysis/scripts$ cat improvement_table.py
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
(analysis_env) vgtu@vgtu-B450M-DS3H:~/Downloads/Harish_Thesis/ros2_ws/analysis/scripts$ 

