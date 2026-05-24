from pathlib import Path
import pandas as pd

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")

OUT_DIR = WS / "analysis" / "results_v6_lighting_sweep"
REGISTRY_PATH = OUT_DIR / "v6_run_registry.csv"

LIGHTINGS = ["bright", "dim", "lowlight"]
TRIALS = ["t01", "t02", "t03"]

ESTIMATORS = {
    "wheel": {
        "folder": "wheel_only",
        "run_prefix": "run_wheel_v6",
        "mode": "wheel",
        "topic": "/odom",
    },
    "ekf": {
        "folder": "ekf",
        "run_prefix": "run_ekf_v6",
        "mode": "ekf",
        "topic": "/odometry/filtered",
    },
    "orbslam3": {
        "folder": "orbslam3",
        "run_prefix": "run_orbslam3_v6",
        "mode": "orb",
        "topic": "/orbslam3/pose",
    },
}

def traj_name(lighting: str, trial: str) -> str:
    if trial == "t01":
        return f"traj_cmd_v6_{lighting}"
    return f"traj_cmd_v6_{lighting}_{trial}"

def run_name(estimator: str, lighting: str, trial: str) -> str:
    prefix = ESTIMATORS[estimator]["run_prefix"]
    if trial == "t01":
        return f"{prefix}_{lighting}"
    return f"{prefix}_{lighting}_{trial}"

rows = []

for lighting in LIGHTINGS:
    for trial in TRIALS:
        master_name = traj_name(lighting, trial)
        master_bag = (
            WS / "bags" / "trajectories" / lighting /
            master_name / master_name
        )

        for estimator, cfg in ESTIMATORS.items():
            rname = run_name(estimator, lighting, trial)
            exp_bag = (
                WS / "bags" / "experiments" / cfg["folder"] /
                lighting / rname / rname
            )

            rows.append({
                "lighting": lighting,
                "trial": trial,
                "estimator": estimator,
                "script_mode": cfg["mode"],
                "run_name": f"{lighting}_{trial}_{estimator}",
                "master_bag": str(master_bag),
                "experiment_bag": str(exp_bag),
                "gt_topic": "/gt/odom",
                "estimator_topic": cfg["topic"],
                "master_exists": master_bag.exists(),
                "experiment_exists": exp_bag.exists(),
            })

df = pd.DataFrame(rows)

OUT_DIR.mkdir(parents=True, exist_ok=True)
df.to_csv(REGISTRY_PATH, index=False)

print(f"\nSaved registry: {REGISTRY_PATH}")
print(f"Rows: {len(df)}")

missing = df[(~df["master_exists"]) | (~df["experiment_exists"])]

if missing.empty:
    print("\nAll master and experiment bag paths exist. Good.")
else:
    print("\nWARNING: Missing paths found:")
    print(missing[[
        "lighting", "trial", "estimator",
        "master_exists", "experiment_exists",
        "master_bag", "experiment_bag"
    ]].to_string(index=False))

print("\nRegistry preview:")
print(df[[
    "lighting", "trial", "estimator",
    "script_mode", "run_name",
    "master_exists", "experiment_exists"
]].to_string(index=False))
