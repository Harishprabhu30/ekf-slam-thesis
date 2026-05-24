#!/usr/bin/env python3
"""
make_phase_wise_analysis_fixed.py

Creates phase-wise ATE RMSE tables and plots for Chapter 4.

Outputs:
- thesis_chapter_4_tables/table_4_3_phase_error_<lighting>_<trial>.csv
- thesis_chapter_4_images/fig_4_9a_phase_error_bright.png
- thesis_chapter_4_images/fig_4_9b_phase_error_dim.png
- thesis_chapter_4_images/fig_4_9c_phase_error_lowlight.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


ROOT = Path("/media/vgtu/New Volume1/Harish_Thesis/ros2_ws/analysis/results_v6_lighting_sweep")

OUT_IMG = ROOT / "thesis_chapter_4_images"
OUT_TAB = ROOT / "thesis_chapter_4_tables"
OUT_IMG.mkdir(parents=True, exist_ok=True)
OUT_TAB.mkdir(parents=True, exist_ok=True)

LIGHTINGS = ["bright", "dim", "lowlight"]
TRIAL = "t03"

ESTIMATORS = ["wheel", "ekf", "orbslam3"]

PHASE_ORDER = [
    "Rest / stop",
    "Square turns",
    "Straight motion",
    "Rotation",
    "Curved motion",
]

PHASE_OBSERVATIONS = {
    "Rest / stop": "Stability/noise",
    "Square turns": "Turning/yaw error",
    "Straight motion": "Translational drift",
    "Rotation": "Orientation difficulty",
    "Curved motion": "Combined motion error",
}


def norm_col(name: str) -> str:
    return (
        name.lower()
        .replace(" ", "")
        .replace("-", "")
        .replace("_", "")
        .replace(".", "")
        .replace("/", "")
    )


def find_col(df, candidates, required=True):
    cols_norm = {norm_col(c): c for c in df.columns}

    for cand in candidates:
        key = norm_col(cand)
        if key in cols_norm:
            return cols_norm[key]

    # partial fallback
    for c in df.columns:
        cn = norm_col(c)
        for cand in candidates:
            if norm_col(cand) in cn:
                return c

    if required:
        raise ValueError(
            f"Could not find any of columns {candidates}. Available columns: {list(df.columns)}"
        )
    return None


def to_seconds(series):
    """Convert timestamp column to seconds robustly."""
    arr = pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)
    arr = arr[np.isfinite(arr)]

    if len(arr) == 0:
        raise ValueError("Time column contains no valid numeric values.")

    median_val = np.nanmedian(np.abs(arr))

    # Nanoseconds
    if median_val > 1e12:
        return pd.to_numeric(series, errors="coerce") / 1e9

    # Microseconds
    if median_val > 1e6:
        return pd.to_numeric(series, errors="coerce") / 1e6

    # Already seconds
    return pd.to_numeric(series, errors="coerce")


def load_aligned_trajectory(lighting, trial, estimator):
    path = ROOT / "aligned" / f"{lighting}_{trial}_{estimator}_aligned.csv"
    if not path.exists():
        raise FileNotFoundError(path)

    df = pd.read_csv(path)

    t_col = find_col(df, ["t_ns", "timestamp_ns", "time_ns", "stamp_ns", "t", "time", "sec"])
    gt_x_col = find_col(df, ["gt_x", "x_gt", "gtx", "x_groundtruth", "groundtruth_x"])
    gt_y_col = find_col(df, ["gt_y", "y_gt", "gty", "y_groundtruth", "groundtruth_y"])

    est_x_col = find_col(df, [
        "est_x_aligned", "aligned_x", "x_aligned",
        "est_x", "x_est", "estimate_x", "x"
    ])
    est_y_col = find_col(df, [
        "est_y_aligned", "aligned_y", "y_aligned",
        "est_y", "y_est", "estimate_y", "y"
    ])

    # Prevent accidentally using GT columns as estimator columns
    if est_x_col == gt_x_col or est_y_col == gt_y_col:
        raise ValueError(
            f"Estimator and GT columns overlap in {path}. "
            f"GT=({gt_x_col},{gt_y_col}), EST=({est_x_col},{est_y_col}). "
            f"Columns available: {list(df.columns)}"
        )

    out = pd.DataFrame()
    out["t_sec"] = to_seconds(df[t_col])
    out["gt_x"] = pd.to_numeric(df[gt_x_col], errors="coerce")
    out["gt_y"] = pd.to_numeric(df[gt_y_col], errors="coerce")
    out["est_x"] = pd.to_numeric(df[est_x_col], errors="coerce")
    out["est_y"] = pd.to_numeric(df[est_y_col], errors="coerce")

    out = out.dropna().reset_index(drop=True)

    # Normalize trajectory time to local zero
    out["t_sec"] = out["t_sec"] - out["t_sec"].min()

    out["ate"] = np.sqrt(
        (out["est_x"] - out["gt_x"]) ** 2 +
        (out["est_y"] - out["gt_y"]) ** 2
    )

    return out


def map_phase_label(value):
    """Map numeric or string phase labels into thesis phase names."""
    if pd.isna(value):
        return None

    s = str(value).strip().lower()

    # Numeric phase IDs
    try:
        v = int(float(s))
        return {
            0: "Rest / stop",
            1: "Square turns",
            2: "Straight motion",
            3: "Rotation",
            4: "Curved motion",
        }.get(v, None)
    except ValueError:
        pass

    # String labels
    if "rest" in s or "stop" in s or "idle" in s:
        return "Rest / stop"
    if "square" in s or "turn" in s:
        return "Square turns"
    if "straight" in s:
        return "Straight motion"
    if "rot" in s or "cw" in s or "clockwise" in s:
        return "Rotation"
    if "curve" in s or "circle" in s or "circular" in s:
        return "Curved motion"

    return None


def load_phase_events(lighting, trial):
    """
    Load phase events from any estimator folder for the same lighting/trial.
    Prefer wheel, then ekf, then orbslam3.
    """
    chosen = None
    for estimator in ESTIMATORS:
        candidate = ROOT / "extracted" / f"{lighting}_{trial}_{estimator}_phase_events.csv"
        if candidate.exists():
            chosen = candidate
            break

    if chosen is None:
        raise FileNotFoundError(f"No phase_events CSV found for {lighting}_{trial}")

    df = pd.read_csv(chosen)

    t_col = find_col(df, ["t_ns", "timestamp_ns", "time_ns", "stamp_ns", "t", "time", "sec"])
    phase_col = find_col(df, ["phase", "phase_id", "phase_name", "label", "event", "data", "value"])

    events = pd.DataFrame()
    events["t_sec"] = to_seconds(df[t_col])
    events["phase"] = df[phase_col].apply(map_phase_label)
    events = events.dropna().sort_values("t_sec").reset_index(drop=True)

    if len(events) == 0:
        raise ValueError(
            f"Phase events were found in {chosen}, but no valid phase labels were mapped. "
            f"Available columns: {list(df.columns)}"
        )

    # Normalize phase-event time to local zero
    events["t_sec"] = events["t_sec"] - events["t_sec"].min()

    return events, chosen


def assign_phases(traj_df, events_df):
    """
    Assign each trajectory timestamp to the latest phase event.
    """
    event_times = events_df["t_sec"].to_numpy()
    event_phases = events_df["phase"].to_numpy()

    traj_times = traj_df["t_sec"].to_numpy()

    idx = np.searchsorted(event_times, traj_times, side="right") - 1
    phase_labels = np.array([None] * len(traj_df), dtype=object)

    valid = idx >= 0
    phase_labels[valid] = event_phases[idx[valid]]

    out = traj_df.copy()
    out["phase"] = phase_labels
    return out


def phase_rmse(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return np.nan
    return float(np.sqrt(np.mean(values ** 2)))


def make_phase_table_for_lighting(lighting, trial):
    events_df, phase_file = load_phase_events(lighting, trial)

    rows = []
    phase_estimator_values = {phase: {} for phase in PHASE_ORDER}

    print(f"\n[{lighting}] Using phase file: {phase_file}")

    for estimator in ESTIMATORS:
        traj = load_aligned_trajectory(lighting, trial, estimator)
        traj = assign_phases(traj, events_df)

        counts = traj["phase"].value_counts(dropna=False).to_dict()
        print(f"[{lighting} {estimator}] phase sample counts: {counts}")

        for phase in PHASE_ORDER:
            phase_rows = traj[traj["phase"] == phase]
            rmse = phase_rmse(phase_rows["ate"].to_numpy())
            phase_estimator_values[phase][estimator] = rmse

    for phase in PHASE_ORDER:
        rows.append({
            "Phase": phase,
            "Wheel ATE RMSE": phase_estimator_values[phase].get("wheel", np.nan),
            "EKF ATE RMSE": phase_estimator_values[phase].get("ekf", np.nan),
            "ORB-SLAM3 ATE RMSE": phase_estimator_values[phase].get("orbslam3", np.nan),
            "Main observation": PHASE_OBSERVATIONS[phase],
        })

    table = pd.DataFrame(rows)

    # Save numeric table
    csv_path = OUT_TAB / f"table_4_3_phase_error_{lighting}_{trial}.csv"
    table.to_csv(csv_path, index=False)

    # Save Word-friendly rounded table
    word_table = table.copy()
    for col in ["Wheel ATE RMSE", "EKF ATE RMSE", "ORB-SLAM3 ATE RMSE"]:
        word_table[col] = word_table[col].apply(
            lambda x: "N/A" if pd.isna(x) else f"{x:.3f}"
        )

    word_csv_path = OUT_TAB / f"table_4_3_phase_error_{lighting}_{trial}_word.csv"
    word_table.to_csv(word_csv_path, index=False)

    return table, word_table


def plot_phase_table(table, lighting):
    labels = table["Phase"].tolist()
    x = np.arange(len(labels))
    width = 0.25

    wheel = table["Wheel ATE RMSE"].to_numpy(dtype=float)
    ekf = table["EKF ATE RMSE"].to_numpy(dtype=float)
    orb = table["ORB-SLAM3 ATE RMSE"].to_numpy(dtype=float)

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.bar(x - width, wheel, width, label="Wheel")
    ax.bar(x, ekf, width, label="EKF")
    ax.bar(x + width, orb, width, label="ORB-SLAM3")

    ax.set_ylabel("ATE RMSE [m]")
    ax.set_xlabel("Motion phase")
    ax.set_title(f"Phase-wise localization error ({lighting}, {TRIAL})")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=25, ha="right")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()

    suffix = {
        "bright": "a",
        "dim": "b",
        "lowlight": "c",
    }[lighting]

    out_path = OUT_IMG / f"fig_4_9{suffix}_phase_error_{lighting}.png"
    fig.savefig(out_path, dpi=300)
    plt.close(fig)

    return out_path


def main():
    for lighting in LIGHTINGS:
        table, word_table = make_phase_table_for_lighting(lighting, TRIAL)
        fig_path = plot_phase_table(table, lighting)

        print(f"\nSaved table:")
        print(OUT_TAB / f"table_4_3_phase_error_{lighting}_{TRIAL}.csv")
        print(OUT_TAB / f"table_4_3_phase_error_{lighting}_{TRIAL}_word.csv")
        print(f"Saved figure:")
        print(fig_path)

    print("\nDone. If only Rest / stop is N/A, that is expected after motion-onset synchronization.")


if __name__ == "__main__":
    main()
