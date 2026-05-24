import sys
import argparse
import subprocess
from pathlib import Path
import pandas as pd

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"
SCRIPTS = ROOT / "scripts"

REGISTRY = ROOT / "v6_run_registry.csv"

EXTRACTED = ROOT / "extracted"
ALIGNED = ROOT / "aligned"
METRICS = ROOT / "metrics"
LOGS = ROOT / "logs"

ALL_METRICS_CSV = METRICS / "v6_all_trial_metrics.csv"

def run_cmd(cmd):
    print("\n" + "=" * 80)
    print(" ".join([str(c) for c in cmd]))
    print("=" * 80)
    subprocess.run(cmd, check=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true", help="Recompute extraction and alignment even if files exist")
    ap.add_argument("--only_estimator", default=None, choices=["wheel", "ekf", "orbslam3"])
    ap.add_argument("--only_lighting", default=None, choices=["bright", "dim", "lowlight"])
    args = ap.parse_args()

    for d in [EXTRACTED, ALIGNED, METRICS, LOGS]:
        d.mkdir(parents=True, exist_ok=True)

    if not REGISTRY.exists():
        raise FileNotFoundError(f"Registry not found: {REGISTRY}")

    df = pd.read_csv(REGISTRY)

    if args.only_estimator:
        df = df[df["estimator"] == args.only_estimator].reset_index(drop=True)

    if args.only_lighting:
        df = df[df["lighting"] == args.only_lighting].reset_index(drop=True)

    print(f"Rows to process: {len(df)}")

    extract_script = SCRIPTS / "01_extract_traj_v6.py"
    sync_script = SCRIPTS / "02_sync_and_align_v6.py"
    metrics_script = SCRIPTS / "03_compute_metrics_v6.py"

    for _, row in df.iterrows():
        lighting = row["lighting"]
        trial = row["trial"]
        estimator = row["estimator"]
        mode = row["script_mode"]
        run_name = row["run_name"]
        bag = row["experiment_bag"]

        print("\n" + "#" * 80)
        print(f"Processing: {run_name}")
        print("#" * 80)

        gt_csv = EXTRACTED / f"{run_name}_gt.csv"
        est_csv = EXTRACTED / f"{run_name}_{estimator}.csv"
        phase_csv = EXTRACTED / f"{run_name}_phase_events.csv"

        aligned_csv = ALIGNED / f"{run_name}_aligned.csv"
        meta_json = ALIGNED / f"{run_name}_sync_meta.json"

        # Extract GT and phase
        if args.force or not gt_csv.exists() or not phase_csv.exists():
            run_cmd([
                sys.executable,
                str(extract_script),
                str(bag),
                "--mode", "gt",
                "--out_csv", str(gt_csv),
                "--phase_out_csv", str(phase_csv),
            ])
        else:
            print(f"[skip] extraction GT exists: {gt_csv}")

        # Extract estimator
        if args.force or not est_csv.exists():
            run_cmd([
                sys.executable,
                str(extract_script),
                str(bag),
                "--mode", str(mode),
                "--out_csv", str(est_csv),
            ])
        else:
            print(f"[skip] extraction estimator exists: {est_csv}")

        # Sync and align
        if args.force or not aligned_csv.exists():
            run_cmd([
                sys.executable,
                str(sync_script),
                "--gt", str(gt_csv),
                "--est", str(est_csv),
                "--out", str(aligned_csv),
                "--meta_json", str(meta_json),
            ])
        else:
            print(f"[skip] aligned exists: {aligned_csv}")

        # Metrics always update one row
        run_cmd([
            sys.executable,
            str(metrics_script),
            "--aligned_csv", str(aligned_csv),
            "--gt_csv", str(gt_csv),
            "--est_csv", str(est_csv),
            "--lighting", str(lighting),
            "--trial", str(trial),
            "--estimator", str(estimator),
            "--run_name", str(run_name),
            "--out_csv", str(ALL_METRICS_CSV),
        ])

    print("\nDONE.")
    print(f"All metrics: {ALL_METRICS_CSV}")

if __name__ == "__main__":
    main()
