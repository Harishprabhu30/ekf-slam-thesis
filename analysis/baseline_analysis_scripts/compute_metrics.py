# compute_metrics.py

import pandas as pd
import numpy as np

def compute_metrics(csv_path):

    df = pd.read_csv(csv_path)

    x0, y0 = df.iloc[0][["x","y"]]
    xT, yT = df.iloc[-1][["x","y"]]

    drift = np.sqrt((xT - x0)**2 + (yT - y0)**2)

    omega_var = np.var(df["omega_z"])

    jerk = np.gradient(df["omega_z"])
    jerk_var = np.var(jerk)

    return {
        "drift": drift,
        "omega_variance": omega_var,
        "jerk_variance": jerk_var
    }

if __name__ == "__main__":
    import sys

    metrics = compute_metrics(sys.argv[1])

    for k,v in metrics.items():
        print(f"{k}: {v:.6f}")
