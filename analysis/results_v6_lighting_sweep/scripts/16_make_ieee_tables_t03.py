from pathlib import Path
import pandas as pd
import numpy as np

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

FINAL = ROOT / "final_reporting_t03"
OUT = ROOT / "paper_tables_t03"
OUT.mkdir(parents=True, exist_ok=True)

MAIN_CSV = FINAL / "table1_main_v6_lighting_mean_std.csv"
ABL_CSV = FINAL / "table2_orbslam3_default_vs_tuned_t03.csv"

if not MAIN_CSV.exists():
    MAIN_CSV = ROOT / "tables" / "v6_paper_summary_table.csv"

if not ABL_CSV.exists():
    ABL_CSV = ROOT / "ablations" / "default_vs_tuned_t03" / "tables" / "orbslam3_default_vs_tuned_t03.csv"

LIGHTING_LABEL = {
    "bright": "Bright",
    "dim": "Dim",
    "lowlight": "Low-light",
}

EST_LABEL = {
    "wheel": "Wheel",
    "ekf": "EKF",
    "orbslam3": "ORB-SLAM3",
}

CONFIG_LABEL = {
    "default_1000_20_7": "Default 1000/20/7",
    "tuned_1800_10_4": "Tuned 1800/10/4",
}

def tex_pm(x):
    if pd.isna(x):
        return "--"
    s = str(x)
    s = s.replace("±", "$\\pm$")
    return s

def fmt(x, nd=3):
    try:
        return f"{float(x):.{nd}f}"
    except Exception:
        return "--"

def improvement(default_val, tuned_val):
    default_val = float(default_val)
    tuned_val = float(tuned_val)
    if abs(default_val) < 1e-12:
        return np.nan
    return 100.0 * (default_val - tuned_val) / default_val

def write(path, text):
    path.write_text(text)
    print(f"Saved: {path}")

def make_main_table():
    df = pd.read_csv(MAIN_CSV)

    # Expected columns from v6_paper_summary_table.csv:
    # lighting, estimator, trials, ate_rmse_m, ate_norm_rmse,
    # drift_rate_m_per_m, yaw_abs_mean_deg, rpe_trans_rmse_m_1.0s, pose_count_ratio

    rows = []

    for _, r in df.iterrows():
        rows.append(
            [
                LIGHTING_LABEL.get(str(r["lighting"]), str(r["lighting"])),
                EST_LABEL.get(str(r["estimator"]), str(r["estimator"])),
                str(int(r["trials"])),
                tex_pm(r["ate_rmse_m"]),
                tex_pm(r["ate_norm_rmse"]),
                tex_pm(r["drift_rate_m_per_m"]),
                tex_pm(r["yaw_abs_mean_deg"]),
                tex_pm(r["rpe_trans_rmse_m_1.0s"]),
                tex_pm(r["pose_count_ratio"]),
            ]
        )

    body = "\n".join(
        " & ".join(row) + r" \\"
        for row in rows
    )

    tex = r"""\begin{table*}[t]
\centering
\caption{Main V6 lighting robustness results. Values are reported as mean $\pm$ standard deviation across three repeated trials. Each estimator output was evaluated against the ground truth from the same trial after motion-onset synchronization and SE(2) alignment without scale correction.}
\label{tab:v6-main-lighting}
\resizebox{\textwidth}{!}{%
\begin{tabular}{llccccccc}
\hline
Lighting & Estimator & Trials & ATE RMSE [m] & Norm. ATE [m/m] & Drift [m/m] & Yaw mean [deg] & RPE$_{1s}$ [m] & Pose output ratio \\
\hline
""" + body + r"""
\hline
\end{tabular}%
}
\end{table*}
"""
    write(OUT / "table1_main_v6_lighting_mean_std.tex", tex)

def make_ablation_table():
    df = pd.read_csv(ABL_CSV)

    rows = []

    for _, r in df.iterrows():
        rows.append(
            [
                LIGHTING_LABEL.get(str(r["lighting"]), str(r["lighting"])),
                CONFIG_LABEL.get(str(r["config"]), str(r["config"])),
                fmt(r["ate_rmse_m"], 3),
                fmt(r["ate_norm_rmse"], 3),
                fmt(r["drift_rate_m_per_m"], 3),
                fmt(r["yaw_abs_mean_deg"], 2),
                fmt(r["rpe_trans_rmse_m_1.0s"], 3),
                fmt(r["pose_count_ratio"], 2),
            ]
        )

    body = "\n".join(
        " & ".join(row) + r" \\"
        for row in rows
    )

    tex = r"""\begin{table}[t]
\centering
\caption{ORB-SLAM3 parameter ablation on selected t03 runs. The default setting uses 1000 features with FAST thresholds 20/7, while the tuned setting uses 1800 features with FAST thresholds 10/4.}
\label{tab:orb-ablation-t03}
\resizebox{\columnwidth}{!}{%
\begin{tabular}{llcccccc}
\hline
Lighting & Configuration & ATE RMSE [m] & Norm. ATE & Drift & Yaw [deg] & RPE$_{1s}$ [m] & Pose ratio \\
\hline
""" + body + r"""
\hline
\end{tabular}%
}
\end{table}
"""
    write(OUT / "table2_orbslam3_default_vs_tuned_t03.tex", tex)

def make_ablation_effect_table():
    df = pd.read_csv(ABL_CSV)

    rows = []

    for lighting in ["bright", "dim", "lowlight"]:
        d = df[(df["lighting"] == lighting) & (df["config"] == "default_1000_20_7")]
        t = df[(df["lighting"] == lighting) & (df["config"] == "tuned_1800_10_4")]

        if d.empty or t.empty:
            continue

        d = d.iloc[0]
        t = t.iloc[0]

        rows.append(
            [
                LIGHTING_LABEL.get(lighting, lighting),
                fmt(improvement(d["ate_rmse_m"], t["ate_rmse_m"]), 1),
                fmt(improvement(d["ate_norm_rmse"], t["ate_norm_rmse"]), 1),
                fmt(improvement(d["yaw_abs_mean_deg"], t["yaw_abs_mean_deg"]), 1),
                fmt(improvement(d["rpe_trans_rmse_m_1.0s"], t["rpe_trans_rmse_m_1.0s"]), 1),
                fmt(float(t["pose_count_ratio"]) - float(d["pose_count_ratio"]), 2),
            ]
        )

    body = "\n".join(
        " & ".join(row) + r" \\"
        for row in rows
    )

    tex = r"""\begin{table}[t]
\centering
\caption{Effect of the tuned ORB-SLAM3 configuration relative to the default configuration on t03 runs. Positive values indicate that the tuned configuration reduced the error metric.}
\label{tab:orb-ablation-effect-t03}
\resizebox{\columnwidth}{!}{%
\begin{tabular}{lccccc}
\hline
Lighting & ATE impr. [\%] & Norm. ATE impr. [\%] & Yaw impr. [\%] & RPE impr. [\%] & Pose ratio change \\
\hline
""" + body + r"""
\hline
\end{tabular}%
}
\end{table}
"""
    write(OUT / "table3_orbslam3_ablation_effectiveness_t03.tex", tex)

def main():
    make_main_table()
    make_ablation_table()
    make_ablation_effect_table()

    print(f"\nIEEE table files saved in: {OUT}")

if __name__ == "__main__":
    main()
