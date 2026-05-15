from pathlib import Path
import subprocess
import sys
import pandas as pd
import numpy as np

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

SCRIPTS = ROOT / "scripts"
MAIN_METRICS = ROOT / "metrics" / "v6_all_trial_metrics.csv"

ABL_ROOT = ROOT / "ablations" / "default_vs_tuned_t01"
EXTRACTED = ABL_ROOT / "extracted"
ALIGNED = ABL_ROOT / "aligned"
METRICS = ABL_ROOT / "metrics"
TABLES = ABL_ROOT / "tables"

for d in [EXTRACTED, ALIGNED, METRICS, TABLES]:
    d.mkdir(parents=True, exist_ok=True)

LIGHTINGS = ["bright", "dim", "lowlight"]

DEFAULT_BAGS = {
    "bright": WS / "bags/experiments/orbslam3_ablation/default/bright/run_orbslam3_default_v6_bright_t01/run_orbslam3_default_v6_bright_t01",
    "dim": WS / "bags/experiments/orbslam3_ablation/default/dim/run_orbslam3_default_v6_dim_t01/run_orbslam3_default_v6_dim_t01",
    "lowlight": WS / "bags/experiments/orbslam3_ablation/default/lowlight/run_orbslam3_default_v6_lowlight_t01/run_orbslam3_default_v6_lowlight_t01",
}

DEFAULT_METRICS_CSV = METRICS / "orbslam3_default_t01_metrics.csv"
COMPARISON_CSV = TABLES / "orbslam3_default_vs_tuned_t01.csv"
EFFECT_CSV = TABLES / "orbslam3_ablation_effectiveness_t01.csv"

def run_cmd(cmd):
    print("\n" + "=" * 80)
    print(" ".join(str(c) for c in cmd))
    print("=" * 80)
    subprocess.run(cmd, check=True)

def analyze_default_runs():
    extract_script = SCRIPTS / "01_extract_traj_v6.py"
    sync_script = SCRIPTS / "02_sync_and_align_v6.py"
    metrics_script = SCRIPTS / "03_compute_metrics_v6.py"

    for lighting in LIGHTINGS:
        bag = DEFAULT_BAGS[lighting]

        if not bag.exists():
            raise FileNotFoundError(f"Missing default ablation bag: {bag}")

        run_name = f"{lighting}_t01_orbslam3_default"

        gt_csv = EXTRACTED / f"{run_name}_gt.csv"
        est_csv = EXTRACTED / f"{run_name}_orbslam3_default.csv"
        phase_csv = EXTRACTED / f"{run_name}_phase_events.csv"
        aligned_csv = ALIGNED / f"{run_name}_aligned.csv"
        meta_json = ALIGNED / f"{run_name}_sync_meta.json"

        print("\n" + "#" * 80)
        print(f"Analyzing default ORB ablation: {run_name}")
        print("#" * 80)

        run_cmd([
            sys.executable, str(extract_script), str(bag),
            "--mode", "gt",
            "--out_csv", str(gt_csv),
            "--phase_out_csv", str(phase_csv),
        ])

        run_cmd([
            sys.executable, str(extract_script), str(bag),
            "--mode", "orb",
            "--out_csv", str(est_csv),
        ])

        run_cmd([
            sys.executable, str(sync_script),
            "--gt", str(gt_csv),
            "--est", str(est_csv),
            "--out", str(aligned_csv),
            "--meta_json", str(meta_json),
        ])

        run_cmd([
            sys.executable, str(metrics_script),
            "--aligned_csv", str(aligned_csv),
            "--gt_csv", str(gt_csv),
            "--est_csv", str(est_csv),
            "--lighting", lighting,
            "--trial", "t01",
            "--estimator", "orbslam3_default",
            "--run_name", run_name,
            "--out_csv", str(DEFAULT_METRICS_CSV),
        ])

def make_comparison_tables():
    if not MAIN_METRICS.exists():
        raise FileNotFoundError(f"Missing main metrics: {MAIN_METRICS}")

    if not DEFAULT_METRICS_CSV.exists():
        raise FileNotFoundError(f"Missing default ablation metrics: {DEFAULT_METRICS_CSV}")

    main = pd.read_csv(MAIN_METRICS)
    default = pd.read_csv(DEFAULT_METRICS_CSV)

    tuned = main[
        (main["estimator"] == "orbslam3") &
        (main["trial"] == "t01") &
        (main["lighting"].isin(LIGHTINGS))
    ].copy()

    tuned["config"] = "tuned_1800_10_4"
    default["config"] = "default_1000_20_7"

    cols = [
        "lighting", "trial", "config",
        "ate_rmse_m",
        "ate_norm_rmse",
        "drift_rate_m_per_m",
        "yaw_abs_mean_deg",
        "rpe_trans_rmse_m_1.0s",
        "pose_count_ratio",
    ]

    comparison = pd.concat([default[cols], tuned[cols]], ignore_index=True)
    comparison["lighting"] = pd.Categorical(comparison["lighting"], categories=LIGHTINGS, ordered=True)
    comparison["config"] = pd.Categorical(
        comparison["config"],
        categories=["default_1000_20_7", "tuned_1800_10_4"],
        ordered=True
    )

    comparison = comparison.sort_values(["lighting", "config"]).reset_index(drop=True)
    comparison.to_csv(COMPARISON_CSV, index=False)

    # Effectiveness table: positive improvement means tuned is better.
    rows = []

    for lighting in LIGHTINGS:
        d = comparison[
            (comparison["lighting"] == lighting) &
            (comparison["config"] == "default_1000_20_7")
        ].iloc[0]

        t = comparison[
            (comparison["lighting"] == lighting) &
            (comparison["config"] == "tuned_1800_10_4")
        ].iloc[0]

        def improvement(default_val, tuned_val):
            if pd.isna(default_val) or abs(default_val) < 1e-12:
                return np.nan
            return 100.0 * (default_val - tuned_val) / default_val

        rows.append({
            "lighting": lighting,
            "trial": "t01",

            "default_ate_rmse_m": float(d["ate_rmse_m"]),
            "tuned_ate_rmse_m": float(t["ate_rmse_m"]),
            "tuned_ate_improvement_pct": improvement(float(d["ate_rmse_m"]), float(t["ate_rmse_m"])),

            "default_norm_ate": float(d["ate_norm_rmse"]),
            "tuned_norm_ate": float(t["ate_norm_rmse"]),
            "tuned_norm_ate_improvement_pct": improvement(float(d["ate_norm_rmse"]), float(t["ate_norm_rmse"])),

            "default_yaw_mean_deg": float(d["yaw_abs_mean_deg"]),
            "tuned_yaw_mean_deg": float(t["yaw_abs_mean_deg"]),
            "tuned_yaw_improvement_pct": improvement(float(d["yaw_abs_mean_deg"]), float(t["yaw_abs_mean_deg"])),

            "default_rpe_trans_rmse_m_1s": float(d["rpe_trans_rmse_m_1.0s"]),
            "tuned_rpe_trans_rmse_m_1s": float(t["rpe_trans_rmse_m_1.0s"]),
            "tuned_rpe_improvement_pct": improvement(float(d["rpe_trans_rmse_m_1.0s"]), float(t["rpe_trans_rmse_m_1.0s"])),

            "default_pose_output_ratio": float(d["pose_count_ratio"]),
            "tuned_pose_output_ratio": float(t["pose_count_ratio"]),
            "pose_output_ratio_change": float(t["pose_count_ratio"]) - float(d["pose_count_ratio"]),
        })

    effect = pd.DataFrame(rows)
    effect.to_csv(EFFECT_CSV, index=False)

    print("\nSaved comparison table:")
    print(COMPARISON_CSV)
    print(comparison.to_string(index=False))

    print("\nSaved effectiveness table:")
    print(EFFECT_CSV)
    print(effect.to_string(index=False))

def main():
    analyze_default_runs()
    make_comparison_tables()

if __name__ == "__main__":
    main()
