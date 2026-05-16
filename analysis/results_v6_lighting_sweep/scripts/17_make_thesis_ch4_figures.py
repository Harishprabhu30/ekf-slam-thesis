#!/usr/bin/env python3
"""
17_make_thesis_ch4_figures.py

Creates thesis Chapter 4 figures from the V6 lighting sweep results.

Input root:
    /media/vgtu/New Volume1/Harish_Thesis/ros2_ws/analysis/results_v6_lighting_sweep

Output folder:
    /media/vgtu/New Volume1/Harish_Thesis/ros2_ws/analysis/results_v6_lighting_sweep/thesis_chapter_4_images

Generated figures:
    fig_4_1a_trajectory_bright_representative.png
    fig_4_1b_trajectory_dim_representative.png
    fig_4_1c_trajectory_lowlight_representative.png

    fig_4_2_ate_rmse_mean_std_by_lighting.png

    fig_4_3a_ate_time_bright_representative.png
    fig_4_3b_ate_time_dim_representative.png
    fig_4_3c_ate_time_lowlight_representative.png

    fig_4_4_rpe_translation_mean_std_by_lighting.png
    fig_4_5_rpe_yaw_mean_std_by_lighting.png  # only if RPE yaw exists

    fig_4_6_yaw_error_mean_std_by_lighting.png
    fig_4_7_yaw_error_time_representative.png

    fig_4_8_drift_rate_mean_std_by_lighting.png

    fig_4_9a_phase_error_bright.png
    fig_4_9b_phase_error_dim.png
    fig_4_9c_phase_error_lowlight.png
    fig_4_9_phase_error_lowlight.png

    fig_4_10_orbslam3_lighting_robustness.png
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# USER PATHS
# ============================================================

ROOT = Path("/media/vgtu/New Volume1/Harish_Thesis/ros2_ws/analysis/results_v6_lighting_sweep")

METRICS_CSV = ROOT / "metrics" / "v6_all_trial_metrics.csv"
ALIGNED_DIR = ROOT / "aligned"
EXTRACTED_DIR = ROOT / "extracted"

OUT_DIR = ROOT / "thesis_chapter_4_images"


LIGHTING_ORDER = ["bright", "dim", "lowlight"]
LIGHTING_LABELS = {
    "bright": "Bright",
    "dim": "Dim",
    "lowlight": "Low-light",
}

ESTIMATOR_ORDER = ["wheel", "ekf", "orbslam3"]
ESTIMATOR_LABELS = {
    "wheel": "Wheel",
    "ekf": "EKF",
    "orbslam3": "ORB-SLAM3",
}


# ============================================================
# GENERAL HELPERS
# ============================================================

def ensure_output_dir() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)


def normalize_col_name(name: str) -> str:
    return str(name).strip().lower().replace("-", "_").replace(" ", "_")


def find_column(df: pd.DataFrame, candidates: List[str], required: bool = True) -> Optional[str]:
    normalized = {normalize_col_name(c): c for c in df.columns}

    for cand in candidates:
        key = normalize_col_name(cand)
        if key in normalized:
            return normalized[key]

    if required:
        raise KeyError(
            f"Missing required column. Tried: {candidates}\n"
            f"Available columns: {list(df.columns)}"
        )

    return None


def save_current_fig(path: Path) -> None:
    plt.tight_layout()
    plt.savefig(path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[SAVED] {path}")


def wrap_to_pi(angle_rad: np.ndarray) -> np.ndarray:
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def standardize_lighting(value: str) -> str:
    s = str(value).strip().lower().replace("-", "").replace("_", "")
    if s in ["bright"]:
        return "bright"
    if s in ["dim"]:
        return "dim"
    if s in ["lowlight", "low"]:
        return "lowlight"
    return s


def standardize_estimator(value: str) -> str:
    s = str(value).strip().lower().replace("-", "").replace("_", "")

    if s in ["wheel", "wheelodom", "wheelodometry", "odom"]:
        return "wheel"

    if s in ["ekf", "ekfbaseline", "odometryfiltered"]:
        return "ekf"

    if s in ["orbslam3", "orbslam", "orb"]:
        return "orbslam3"

    return s


# ============================================================
# LOAD MAIN METRICS
# ============================================================

def load_metrics() -> pd.DataFrame:
    if not METRICS_CSV.exists():
        raise FileNotFoundError(f"Metrics CSV not found: {METRICS_CSV}")

    df = pd.read_csv(METRICS_CSV)

    lighting_col = find_column(df, ["lighting", "light", "condition"])
    trial_col = find_column(df, ["trial", "run", "trial_id"])
    estimator_col = find_column(df, ["estimator", "config", "method", "system"])

    ate_col = find_column(df, ["ate_rmse_m", "ate_rmse", "ate"])
    norm_ate_col = find_column(df, ["ate_norm_rmse", "normalized_ate", "norm_ate", "ate_norm"], required=False)
    drift_col = find_column(df, ["drift_rate_m_per_m", "drift_rate", "drift"], required=False)
    yaw_col = find_column(df, ["yaw_abs_mean_deg", "yaw_mean_abs_deg", "yaw_error_deg"], required=False)
    rpe_col = find_column(df, ["rpe_trans_rmse_m_1.0s", "rpe_trans_rmse_m", "rpe_translation_rmse", "rpe_1s"], required=False)
    pose_ratio_col = find_column(df, ["pose_count_ratio", "pose_ratio", "ratio"], required=False)
    rpe_yaw_col = find_column(df, ["rpe_yaw_rmse_deg_1.0s", "rpe_yaw_rmse_deg", "rpe_yaw"], required=False)

    out = pd.DataFrame()
    out["lighting"] = df[lighting_col].apply(standardize_lighting)
    out["trial"] = df[trial_col].astype(str)
    out["estimator"] = df[estimator_col].apply(standardize_estimator)

    out["ate_rmse_m"] = pd.to_numeric(df[ate_col], errors="coerce")

    if norm_ate_col:
        out["ate_norm_rmse"] = pd.to_numeric(df[norm_ate_col], errors="coerce")
    else:
        out["ate_norm_rmse"] = np.nan

    if drift_col:
        out["drift_rate_m_per_m"] = pd.to_numeric(df[drift_col], errors="coerce")
    else:
        out["drift_rate_m_per_m"] = np.nan

    if yaw_col:
        out["yaw_abs_mean_deg"] = pd.to_numeric(df[yaw_col], errors="coerce")
    else:
        out["yaw_abs_mean_deg"] = np.nan

    if rpe_col:
        out["rpe_trans_rmse_m_1.0s"] = pd.to_numeric(df[rpe_col], errors="coerce")
    else:
        out["rpe_trans_rmse_m_1.0s"] = np.nan

    if pose_ratio_col:
        out["pose_count_ratio"] = pd.to_numeric(df[pose_ratio_col], errors="coerce")
    else:
        out["pose_count_ratio"] = np.nan

    if rpe_yaw_col:
        out["rpe_yaw_rmse_deg_1.0s"] = pd.to_numeric(df[rpe_yaw_col], errors="coerce")
    else:
        out["rpe_yaw_rmse_deg_1.0s"] = np.nan

    out = out[out["lighting"].isin(LIGHTING_ORDER)]
    out = out[out["estimator"].isin(ESTIMATOR_ORDER)]

    print("\n[INFO] Loaded metrics:")
    print(out.head())
    print(f"[INFO] Metrics rows: {len(out)}")

    return out


# ============================================================
# LOAD ALIGNED TRAJECTORY FILES
# ============================================================

def aligned_csv_path(lighting: str, trial: str, estimator: str) -> Path:
    return ALIGNED_DIR / f"{lighting}_{trial}_{estimator}_aligned.csv"


def sync_meta_path(lighting: str, trial: str, estimator: str) -> Path:
    return ALIGNED_DIR / f"{lighting}_{trial}_{estimator}_sync_meta.json"


def phase_events_path(lighting: str, trial: str, estimator: str) -> Path:
    return EXTRACTED_DIR / f"{lighting}_{trial}_{estimator}_phase_events.csv"


def read_aligned_csv(lighting: str, trial: str, estimator: str) -> pd.DataFrame:
    path = aligned_csv_path(lighting, trial, estimator)

    if not path.exists():
        raise FileNotFoundError(f"Aligned CSV not found: {path}")

    raw = pd.read_csv(path)

    time_col = find_column(
        raw,
        ["t_s", "time_s", "time", "timestamp_s", "stamp_s", "t"],
        required=False,
    )

    time_ns_col = find_column(
        raw,
        ["t_ns", "time_ns", "timestamp_ns", "stamp_ns"],
        required=False,
    )

    if time_col:
        time_s = pd.to_numeric(raw[time_col], errors="coerce")
    elif time_ns_col:
        time_s = pd.to_numeric(raw[time_ns_col], errors="coerce") / 1e9
    else:
        time_s = pd.Series(np.arange(len(raw), dtype=float))

    gt_x_col = find_column(raw, ["gt_x", "x_gt", "ref_x", "gt_px"])
    gt_y_col = find_column(raw, ["gt_y", "y_gt", "ref_y", "gt_py"])

    est_x_col = find_column(
        raw,
        ["est_x", "x_est", "aligned_x", "x_aligned", "traj_x", "x"],
    )
    est_y_col = find_column(
        raw,
        ["est_y", "y_est", "aligned_y", "y_aligned", "traj_y", "y"],
    )

    out = pd.DataFrame()
    out["time_s"] = pd.to_numeric(time_s, errors="coerce")
    out["gt_x"] = pd.to_numeric(raw[gt_x_col], errors="coerce")
    out["gt_y"] = pd.to_numeric(raw[gt_y_col], errors="coerce")
    out["est_x"] = pd.to_numeric(raw[est_x_col], errors="coerce")
    out["est_y"] = pd.to_numeric(raw[est_y_col], errors="coerce")

    ate_col = find_column(
        raw,
        ["ate_m", "ate", "position_error_m", "pos_error_m", "trans_error_m"],
        required=False,
    )

    if ate_col:
        out["ate_m"] = pd.to_numeric(raw[ate_col], errors="coerce")
    else:
        out["ate_m"] = np.sqrt(
            (out["est_x"] - out["gt_x"]) ** 2 +
            (out["est_y"] - out["gt_y"]) ** 2
        )

    gt_yaw_col = find_column(
        raw,
        ["gt_yaw", "yaw_gt", "gt_yaw_rad", "ref_yaw"],
        required=False,
    )

    est_yaw_col = find_column(
        raw,
        ["est_yaw", "yaw_est", "aligned_yaw", "yaw_aligned", "yaw"],
        required=False,
    )

    yaw_error_col = find_column(
        raw,
        ["yaw_error_deg", "yaw_abs_error_deg", "yaw_error", "yaw_err"],
        required=False,
    )

    if yaw_error_col:
        yaw_error_raw = pd.to_numeric(raw[yaw_error_col], errors="coerce").to_numpy()

        # If values look like radians, convert to degrees.
        if np.nanmax(np.abs(yaw_error_raw)) <= 3.5:
            out["yaw_error_deg"] = np.degrees(np.abs(wrap_to_pi(yaw_error_raw)))
        else:
            out["yaw_error_deg"] = np.abs(yaw_error_raw)

    elif gt_yaw_col and est_yaw_col:
        gt_yaw = pd.to_numeric(raw[gt_yaw_col], errors="coerce").to_numpy()
        est_yaw = pd.to_numeric(raw[est_yaw_col], errors="coerce").to_numpy()

        # If values look like degrees, wrap in degrees.
        if np.nanmax(np.abs(gt_yaw)) > 3.5 or np.nanmax(np.abs(est_yaw)) > 3.5:
            out["yaw_error_deg"] = np.abs(((gt_yaw - est_yaw + 180.0) % 360.0) - 180.0)
        else:
            out["yaw_error_deg"] = np.degrees(np.abs(wrap_to_pi(gt_yaw - est_yaw)))
    else:
        out["yaw_error_deg"] = np.nan

    out = out.dropna(subset=["time_s", "gt_x", "gt_y", "est_x", "est_y"]).copy()
    out = out.sort_values("time_s")

    if len(out) > 0:
        out["time_s"] = out["time_s"] - out["time_s"].iloc[0]

    return out


# ============================================================
# REPRESENTATIVE TRIAL SELECTION
# ============================================================

def choose_representative_trial(metrics: pd.DataFrame, lighting: str) -> str:
    """
    Representative trial is the median ORB-SLAM3 trial by ATE RMSE.
    This is suitable because ORB-SLAM3 is the lighting-sensitive estimator.
    """
    sub = metrics[
        (metrics["lighting"] == lighting) &
        (metrics["estimator"] == "orbslam3")
    ].copy()

    if sub.empty:
        raise ValueError(f"No ORB-SLAM3 metrics found for lighting: {lighting}")

    sub = sub.sort_values("ate_rmse_m").reset_index(drop=True)
    median_idx = len(sub) // 2
    trial = str(sub.loc[median_idx, "trial"])

    print(
        f"[INFO] Representative trial for {lighting}: {trial} "
        f"(median ORB-SLAM3 ATE RMSE)"
    )

    return trial


def get_representative_trials(metrics: pd.DataFrame) -> Dict[str, str]:
    reps = {
        lighting: choose_representative_trial(metrics, lighting)
        for lighting in LIGHTING_ORDER
    }

    # Force low-light representative trial for thesis because t03 is the key example.
    reps["lowlight"] = "t03"

    return reps

# ============================================================
# SUMMARY BAR PLOTS
# ============================================================

def plot_grouped_bar(
    metrics: pd.DataFrame,
    metric_col: str,
    ylabel: str,
    title: str,
    out_name: str,
) -> None:
    summary = (
        metrics
        .groupby(["lighting", "estimator"])[metric_col]
        .agg(["mean", "std"])
        .reset_index()
    )

    x = np.arange(len(LIGHTING_ORDER))
    width = 0.24

    plt.figure(figsize=(8.6, 5.0))

    for idx, estimator in enumerate(ESTIMATOR_ORDER):
        est_data = summary[summary["estimator"] == estimator].set_index("lighting")

        means = []
        stds = []

        for lighting in LIGHTING_ORDER:
            if lighting in est_data.index:
                means.append(est_data.loc[lighting, "mean"])
                stds.append(est_data.loc[lighting, "std"])
            else:
                means.append(np.nan)
                stds.append(np.nan)

        plt.bar(
            x + (idx - 1) * width,
            means,
            width=width,
            yerr=stds,
            capsize=4,
            label=ESTIMATOR_LABELS[estimator],
        )

    plt.xticks(x, [LIGHTING_LABELS[l] for l in LIGHTING_ORDER])
    plt.xlabel("Lighting condition")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend()

    save_current_fig(OUT_DIR / out_name)


def make_summary_plots(metrics: pd.DataFrame) -> None:
    plot_grouped_bar(
        metrics,
        metric_col="ate_rmse_m",
        ylabel="ATE RMSE [m]",
        title="ATE RMSE by lighting condition",
        out_name="fig_4_2_ate_rmse_mean_std_by_lighting.png",
    )

    plot_grouped_bar(
        metrics,
        metric_col="rpe_trans_rmse_m_1.0s",
        ylabel="RPE translation RMSE [m]",
        title="RPE translation error by lighting condition",
        out_name="fig_4_4_rpe_translation_mean_std_by_lighting.png",
    )

    if metrics["rpe_yaw_rmse_deg_1.0s"].notna().any():
        plot_grouped_bar(
            metrics,
            metric_col="rpe_yaw_rmse_deg_1.0s",
            ylabel="RPE yaw RMSE [deg]",
            title="RPE yaw error by lighting condition",
            out_name="fig_4_5_rpe_yaw_mean_std_by_lighting.png",
        )
    else:
        print("[SKIP] RPE yaw plot skipped because no RPE yaw column/data was found.")

    plot_grouped_bar(
        metrics,
        metric_col="yaw_abs_mean_deg",
        ylabel="Mean absolute yaw error [deg]",
        title="Yaw error by lighting condition",
        out_name="fig_4_6_yaw_error_mean_std_by_lighting.png",
    )

    plot_grouped_bar(
        metrics,
        metric_col="drift_rate_m_per_m",
        ylabel="Drift rate [m/m]",
        title="Drift rate by lighting condition",
        out_name="fig_4_8_drift_rate_mean_std_by_lighting.png",
    )


# ============================================================
# REPRESENTATIVE TRAJECTORY AND TIME PLOTS
# ============================================================

def load_representative_set(lighting: str, trial: str) -> Dict[str, pd.DataFrame]:
    data = {}

    for estimator in ESTIMATOR_ORDER:
        try:
            data[estimator] = read_aligned_csv(lighting, trial, estimator)
        except Exception as exc:
            print(f"[WARN] Could not load {lighting}_{trial}_{estimator}: {exc}")

    return data


def plot_representative_trajectory(lighting: str, trial: str, data: Dict[str, pd.DataFrame], out_name: str) -> None:
    if not data:
        print(f"[SKIP] No data for trajectory plot: {lighting} {trial}")
        return

    plt.figure(figsize=(7.2, 6.0))

    # Use GT from first available estimator file.
    first_df = next(iter(data.values()))
    plt.plot(
        first_df["gt_x"],
        first_df["gt_y"],
        linewidth=2.4,
        label="GT",
    )

    for estimator in ESTIMATOR_ORDER:
        if estimator not in data:
            continue

        df = data[estimator]
        plt.plot(
            df["est_x"],
            df["est_y"],
            linewidth=1.8,
            label=ESTIMATOR_LABELS[estimator],
        )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"Representative trajectory comparison: {LIGHTING_LABELS[lighting]} ({trial})")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()

    save_current_fig(OUT_DIR / out_name)


def plot_ate_time(lighting: str, trial: str, data: Dict[str, pd.DataFrame], out_name: str) -> None:
    if not data:
        print(f"[SKIP] No data for ATE-time plot: {lighting} {trial}")
        return

    plt.figure(figsize=(8.6, 4.8))

    for estimator in ESTIMATOR_ORDER:
        if estimator not in data:
            continue

        df = data[estimator]
        plt.plot(
            df["time_s"],
            df["ate_m"],
            linewidth=1.8,
            label=ESTIMATOR_LABELS[estimator],
        )

    plt.xlabel("Time [s]")
    plt.ylabel("ATE [m]")
    plt.title(f"ATE over time: {LIGHTING_LABELS[lighting]} representative trial ({trial})")
    plt.grid(True, alpha=0.3)
    plt.legend()

    save_current_fig(OUT_DIR / out_name)


def plot_yaw_error_time(lighting: str, trial: str, data: Dict[str, pd.DataFrame], out_name: str) -> None:
    if not data:
        print(f"[SKIP] No data for yaw-time plot: {lighting} {trial}")
        return

    plt.figure(figsize=(8.6, 4.8))

    any_yaw = False

    for estimator in ESTIMATOR_ORDER:
        if estimator not in data:
            continue

        df = data[estimator]

        if df["yaw_error_deg"].notna().any():
            any_yaw = True
            plt.plot(
                df["time_s"],
                df["yaw_error_deg"],
                linewidth=1.8,
                label=ESTIMATOR_LABELS[estimator],
            )

    if not any_yaw:
        plt.close()
        print(f"[SKIP] No yaw error data found for: {lighting} {trial}")
        return

    plt.xlabel("Time [s]")
    plt.ylabel("Absolute yaw error [deg]")
    plt.title(f"Yaw error over time: {LIGHTING_LABELS[lighting]} representative trial ({trial})")
    plt.grid(True, alpha=0.3)
    plt.legend()

    save_current_fig(OUT_DIR / out_name)


def make_representative_plots(metrics: pd.DataFrame) -> Dict[str, str]:
    reps = get_representative_trials(metrics)

    file_suffix = {
        "bright": "bright",
        "dim": "dim",
        "lowlight": "lowlight",
    }

    letters = {
        "bright": "a",
        "dim": "b",
        "lowlight": "c",
    }

    for lighting in LIGHTING_ORDER:
        trial = reps[lighting]
        data = load_representative_set(lighting, trial)

        plot_representative_trajectory(
            lighting,
            trial,
            data,
            f"fig_4_1{letters[lighting]}_trajectory_{file_suffix[lighting]}_representative.png",
        )

        plot_ate_time(
            lighting,
            trial,
            data,
            f"fig_4_3{letters[lighting]}_ate_time_{file_suffix[lighting]}_representative.png",
        )

    # Use low-light representative for yaw time plot by default.
    lowlight_trial = reps["lowlight"]
    lowlight_data = load_representative_set("lowlight", lowlight_trial)

    plot_yaw_error_time(
        "lowlight",
        lowlight_trial,
        lowlight_data,
        "fig_4_7_yaw_error_time_representative.png",
    )

    return reps


# ============================================================
# PHASE-BASED ANALYSIS
# ============================================================

def read_phase_events(lighting: str, trial: str, estimator: str) -> Optional[pd.DataFrame]:
    path = phase_events_path(lighting, trial, estimator)

    if not path.exists():
        return None

    try:
        raw = pd.read_csv(path)
    except Exception as exc:
        print(f"[WARN] Could not read phase file {path}: {exc}")
        return None

    phase_col = find_column(raw, ["phase", "traj_phase", "label", "data"], required=False)
    if not phase_col:
        return None

    time_s_col = find_column(raw, ["t_s", "time_s", "time", "stamp_s"], required=False)
    time_ns_col = find_column(raw, ["t_ns", "time_ns", "stamp_ns"], required=False)

    if time_s_col:
        t = pd.to_numeric(raw[time_s_col], errors="coerce")
    elif time_ns_col:
        t = pd.to_numeric(raw[time_ns_col], errors="coerce") / 1e9
    else:
        return None

    out = pd.DataFrame()
    out["time_s_raw"] = t
    out["phase"] = raw[phase_col].astype(str)

    out = out.dropna(subset=["time_s_raw"]).sort_values("time_s_raw").reset_index(drop=True)

    if len(out) == 0:
        return None

    return out


def assign_phase_to_aligned_data(aligned: pd.DataFrame, phases: pd.DataFrame) -> pd.DataFrame:
    """
    Assign phase labels to aligned trajectory samples.

    Since phase event timestamps may be absolute while aligned time may be relative,
    this function uses a simple robust approach:
        - if time ranges overlap, use direct phase times
        - otherwise, shift phase events so the first event starts at t=0
    """
    df = aligned.copy()

    phase_times = phases["time_s_raw"].to_numpy(dtype=float)
    phase_labels = phases["phase"].astype(str).to_numpy()

    aligned_min = float(df["time_s"].min())
    aligned_max = float(df["time_s"].max())

    phase_min = float(np.nanmin(phase_times))
    phase_max = float(np.nanmax(phase_times))

    # If phase times do not overlap with aligned relative time, shift them.
    overlap = not (phase_max < aligned_min or phase_min > aligned_max)

    if not overlap:
        phase_times = phase_times - phase_times[0]

    assigned = []

    for t in df["time_s"].to_numpy(dtype=float):
        idx = np.searchsorted(phase_times, t, side="right") - 1
        if idx < 0:
            assigned.append("unknown")
        else:
            assigned.append(phase_labels[idx])

    df["phase"] = assigned

    return df


def phase_order_key(phase: str) -> int:
    p = phase.lower()

    order = [
        "rest",
        "start",
        "square",
        "straight",
        "clockwise",
        "rotation",
        "circle",
        "curved",
        "curve",
        "stop",
    ]

    for i, token in enumerate(order):
        if token in p:
            return i

    return 999


def compute_phase_summary_for_lighting(lighting: str) -> pd.DataFrame:
    rows = []

    for trial in ["t01", "t02", "t03"]:
        for estimator in ESTIMATOR_ORDER:
            aligned_path = aligned_csv_path(lighting, trial, estimator)

            if not aligned_path.exists():
                continue

            try:
                aligned = read_aligned_csv(lighting, trial, estimator)
            except Exception as exc:
                print(f"[WARN] Could not read aligned for phase: {lighting} {trial} {estimator}: {exc}")
                continue

            phases = read_phase_events(lighting, trial, estimator)

            if phases is None:
                continue

            phased = assign_phase_to_aligned_data(aligned, phases)
            phased = phased[phased["phase"] != "unknown"].copy()

            if phased.empty:
                continue

            phase_group = (
                phased
                .groupby("phase")["ate_m"]
                .mean()
                .reset_index()
            )

            for _, r in phase_group.iterrows():
                rows.append(
                    {
                        "lighting": lighting,
                        "trial": trial,
                        "estimator": estimator,
                        "phase": r["phase"],
                        "mean_ate_m": r["ate_m"],
                    }
                )

    if not rows:
        return pd.DataFrame()

    df = pd.DataFrame(rows)

    summary = (
        df
        .groupby(["lighting", "estimator", "phase"])["mean_ate_m"]
        .mean()
        .reset_index()
    )

    summary["phase_order"] = summary["phase"].apply(phase_order_key)
    summary = summary.sort_values(["phase_order", "phase", "estimator"])

    return summary


def plot_phase_summary(lighting: str, out_name: str) -> None:
    phase_df = compute_phase_summary_for_lighting(lighting)

    if phase_df.empty:
        print(f"[SKIP] No phase data available for {lighting}.")
        return

    phases = (
        phase_df[["phase", "phase_order"]]
        .drop_duplicates()
        .sort_values(["phase_order", "phase"])["phase"]
        .tolist()
    )

    x = np.arange(len(phases))
    width = 0.24

    plt.figure(figsize=(10.0, 5.0))

    for idx, estimator in enumerate(ESTIMATOR_ORDER):
        sub = phase_df[phase_df["estimator"] == estimator].set_index("phase")

        means = []
        for phase in phases:
            if phase in sub.index:
                means.append(sub.loc[phase, "mean_ate_m"])
            else:
                means.append(np.nan)

        plt.bar(
            x + (idx - 1) * width,
            means,
            width=width,
            label=ESTIMATOR_LABELS[estimator],
        )

    plt.xticks(x, phases, rotation=25, ha="right")
    plt.xlabel("Motion phase")
    plt.ylabel("Mean ATE [m]")
    plt.title(f"Phase-based mean ATE: {LIGHTING_LABELS[lighting]}")
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend()

    save_current_fig(OUT_DIR / out_name)


def make_phase_plots() -> None:
    plot_phase_summary("bright", "fig_4_9a_phase_error_bright.png")
    plot_phase_summary("dim", "fig_4_9b_phase_error_dim.png")
    plot_phase_summary("lowlight", "fig_4_9c_phase_error_lowlight.png")

    lowlight_path = OUT_DIR / "fig_4_9c_phase_error_lowlight.png"
    if lowlight_path.exists():
        shutil.copyfile(lowlight_path, OUT_DIR / "fig_4_9_phase_error_lowlight.png")
        print(f"[SAVED] {OUT_DIR / 'fig_4_9_phase_error_lowlight.png'}")


# ============================================================
# ORB-SLAM3 LIGHTING ROBUSTNESS
# ============================================================

def make_orbslam3_lighting_robustness(metrics: pd.DataFrame) -> None:
    orb = metrics[metrics["estimator"] == "orbslam3"].copy()

    if orb.empty:
        print("[SKIP] No ORB-SLAM3 data found.")
        return

    summary = (
        orb
        .groupby("lighting")
        .agg(
            ate_mean=("ate_rmse_m", "mean"),
            ate_std=("ate_rmse_m", "std"),
            rpe_mean=("rpe_trans_rmse_m_1.0s", "mean"),
            rpe_std=("rpe_trans_rmse_m_1.0s", "std"),
            ratio_mean=("pose_count_ratio", "mean"),
            ratio_std=("pose_count_ratio", "std"),
        )
        .reindex(LIGHTING_ORDER)
    )

    x = np.arange(len(LIGHTING_ORDER))
    labels = [LIGHTING_LABELS[l] for l in LIGHTING_ORDER]

    fig, axes = plt.subplots(3, 1, figsize=(8.2, 9.0), sharex=True)

    axes[0].bar(x, summary["ate_mean"], yerr=summary["ate_std"], capsize=4)
    axes[0].set_ylabel("ATE RMSE [m]")
    axes[0].set_title("ORB-SLAM3 lighting robustness")
    axes[0].grid(True, axis="y", alpha=0.3)

    axes[1].bar(x, summary["rpe_mean"], yerr=summary["rpe_std"], capsize=4)
    axes[1].set_ylabel("RPE$_{1s}$ [m]")
    axes[1].grid(True, axis="y", alpha=0.3)

    axes[2].bar(x, summary["ratio_mean"], yerr=summary["ratio_std"], capsize=4)
    axes[2].set_ylabel("Pose ratio")
    axes[2].set_xlabel("Lighting condition")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels)
    axes[2].grid(True, axis="y", alpha=0.3)

    plt.tight_layout()
    out_path = OUT_DIR / "fig_4_10_orbslam3_lighting_robustness.png"
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)

    print(f"[SAVED] {out_path}")


# ============================================================
# OPTIONAL: ORB-SLAM3 ABLATION FIGURE FOR THESIS
# ============================================================

def make_orbslam3_ablation_t03_plot() -> None:
    """
    Uses:
        final_reporting_t03/table2_orbslam3_default_vs_tuned_t03.csv

    Saves:
        fig_4_11_orbslam3_default_vs_tuned_t03_ate.png

    This is optional, but useful for the thesis section discussing parameter sensitivity.
    """
    ablation_csv = ROOT / "final_reporting_t03" / "table2_orbslam3_default_vs_tuned_t03.csv"

    if not ablation_csv.exists():
        print(f"[SKIP] Ablation table not found: {ablation_csv}")
        return

    df = pd.read_csv(ablation_csv)

    lighting_col = find_column(df, ["lighting", "light", "condition"])
    config_col = find_column(df, ["config", "configuration", "setting"])
    ate_col = find_column(df, ["ate_rmse_m", "ate", "ate_rmse"])

    df["lighting_std"] = df[lighting_col].apply(standardize_lighting)
    df["config_std"] = df[config_col].astype(str).str.lower()
    df["ate"] = pd.to_numeric(df[ate_col], errors="coerce")

    x = np.arange(len(LIGHTING_ORDER))
    width = 0.35

    plt.figure(figsize=(8.0, 4.8))

    for idx, config in enumerate(["default", "tuned"]):
        sub = df[df["config_std"].str.contains(config)].set_index("lighting_std")

        values = []
        for lighting in LIGHTING_ORDER:
            if lighting in sub.index:
                values.append(sub.loc[lighting, "ate"])
            else:
                values.append(np.nan)

        plt.bar(
            x + (idx - 0.5) * width,
            values,
            width=width,
            label=config.capitalize(),
        )

    plt.xticks(x, [LIGHTING_LABELS[l] for l in LIGHTING_ORDER])
    plt.xlabel("Lighting condition")
    plt.ylabel("ATE RMSE [m]")
    plt.title("ORB-SLAM3 default vs tuned extractor on selected t03 runs")
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend()

    save_current_fig(OUT_DIR / "fig_4_11_orbslam3_default_vs_tuned_t03_ate.png")


# ============================================================
# MAIN
# ============================================================

def main() -> None:
    ensure_output_dir()

    print("\n============================================================")
    print("THESIS CHAPTER 4 FIGURE GENERATION")
    print("============================================================")
    print(f"[ROOT] {ROOT}")
    print(f"[OUT ] {OUT_DIR}")

    metrics = load_metrics()

    print("\n[STEP 1] Creating summary plots over all 27 runs...")
    make_summary_plots(metrics)

    print("\n[STEP 2] Creating representative trajectory and time-series plots...")
    reps = make_representative_plots(metrics)

    print("\n[STEP 3] Creating phase-based plots...")
    make_phase_plots()

    print("\n[STEP 4] Creating ORB-SLAM3 lighting robustness plot...")
    make_orbslam3_lighting_robustness(metrics)

    print("\n[STEP 5] Creating optional ORB-SLAM3 ablation thesis plot...")
    make_orbslam3_ablation_t03_plot()

    print("\n============================================================")
    print("[DONE] Chapter 4 figures generated.")
    print("Representative trials used:")
    for lighting, trial in reps.items():
        print(f"  {LIGHTING_LABELS[lighting]}: {trial}")
    print(f"Output folder: {OUT_DIR}")
    print("============================================================\n")


if __name__ == "__main__":
    main()
