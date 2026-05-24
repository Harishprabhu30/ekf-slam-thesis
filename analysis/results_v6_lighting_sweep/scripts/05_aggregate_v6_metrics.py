from pathlib import Path
import pandas as pd
import numpy as np

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

IN_CSV = ROOT / "metrics" / "v6_all_trial_metrics.csv"
OUT_CSV = ROOT / "tables" / "v6_mean_std_by_lighting_estimator.csv"
PAPER_CSV = ROOT / "tables" / "v6_paper_summary_table.csv"

LIGHTING_ORDER = ["bright", "dim", "lowlight"]
ESTIMATOR_ORDER = ["wheel", "ekf", "orbslam3"]

METRICS = [
    "ate_rmse_m",
    "ate_mean_m",
    "ate_max_m",
    "ate_norm_rmse",
    "drift_rate_m_per_m",
    "yaw_abs_mean_deg",
    "yaw_abs_max_deg",
    "rpe_trans_rmse_m_1.0s",
    "rpe_yaw_rmse_deg_1.0s",
    "pose_output_ratio",
    "gt_path_length_m",
    "duration_s",
]

PAPER_METRICS = [
    "ate_rmse_m",
    "ate_norm_rmse",
    "drift_rate_m_per_m",
    "yaw_abs_mean_deg",
    "rpe_trans_rmse_m_1.0s",
    "pose_output_ratio",
]

def fmt(mean, std, nd=3):
    if pd.isna(mean):
        return ""
    if pd.isna(std):
        std = 0.0
    return f"{mean:.{nd}f} ± {std:.{nd}f}"

def main():
    if not IN_CSV.exists():
        raise FileNotFoundError(f"Metrics CSV not found: {IN_CSV}")

    df = pd.read_csv(IN_CSV)

    if df.empty:
        raise RuntimeError("Metrics CSV is empty")

    available_metrics = [m for m in METRICS if m in df.columns]

    rows = []

    grouped = df.groupby(["lighting", "estimator"], dropna=False)

    for (lighting, estimator), g in grouped:
        out = {
            "lighting": lighting,
            "estimator": estimator,
            "trials": int(g["trial"].nunique()),
            "runs": int(len(g)),
        }

        for m in available_metrics:
            vals = pd.to_numeric(g[m], errors="coerce")
            out[f"{m}_mean"] = float(vals.mean())
            out[f"{m}_std"] = float(vals.std(ddof=1)) if len(vals.dropna()) > 1 else 0.0
            out[f"{m}_min"] = float(vals.min())
            out[f"{m}_max"] = float(vals.max())

        rows.append(out)

    summary = pd.DataFrame(rows)

    summary["lighting"] = pd.Categorical(summary["lighting"], categories=LIGHTING_ORDER, ordered=True)
    summary["estimator"] = pd.Categorical(summary["estimator"], categories=ESTIMATOR_ORDER, ordered=True)
    summary = summary.sort_values(["lighting", "estimator"]).reset_index(drop=True)

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    summary.to_csv(OUT_CSV, index=False)

    # Paper-friendly compact table
    paper_rows = []

    for _, r in summary.iterrows():
        pr = {
            "lighting": str(r["lighting"]),
            "estimator": str(r["estimator"]),
            "trials": int(r["trials"]),
        }

        for m in PAPER_METRICS:
            mean_col = f"{m}_mean"
            std_col = f"{m}_std"

            if mean_col in summary.columns:
                nd = 3
                if m == "pose_count_ratio":
                    nd = 2
                pr[m] = fmt(r[mean_col], r[std_col], nd=nd)

        paper_rows.append(pr)

    paper = pd.DataFrame(paper_rows)
    paper.to_csv(PAPER_CSV, index=False)

    print(f"\nSaved full summary: {OUT_CSV}")
    print(f"Saved paper table:  {PAPER_CSV}")

    print("\nPaper summary table:")
    print(paper.to_string(index=False))

if __name__ == "__main__":
    main()
