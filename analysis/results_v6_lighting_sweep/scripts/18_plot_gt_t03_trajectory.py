#!/usr/bin/env python3

from pathlib import Path
import argparse
import pandas as pd
import matplotlib.pyplot as plt


LIGHTING_ORDER = ["bright", "dim", "lowlight"]
TRIAL = "t03"


def find_xy_columns(df: pd.DataFrame):
    candidates = [
        ("x", "y"),
        ("gt_x", "gt_y"),
        ("x_gt", "y_gt"),
        ("pos_x", "pos_y"),
        ("position_x", "position_y"),
    ]

    for x_col, y_col in candidates:
        if x_col in df.columns and y_col in df.columns:
            return x_col, y_col

    raise ValueError(
        f"Could not find x/y columns. Available columns are:\n{list(df.columns)}"
    )


def load_gt_csv(base_dir: Path, lighting: str, trial: str) -> pd.DataFrame:
    extracted_dir = base_dir / "extracted"

    # Use wheel GT first because it exists for every lighting/trial and is independent of estimator.
    preferred = extracted_dir / f"{lighting}_{trial}_wheel_gt.csv"

    if preferred.exists():
        return pd.read_csv(preferred)

    # Fallback: use any matching GT file for that lighting/trial.
    matches = sorted(extracted_dir.glob(f"{lighting}_{trial}_*_gt.csv"))

    if not matches:
        raise FileNotFoundError(
            f"No GT file found for {lighting} {trial} inside:\n{extracted_dir}"
        )

    print(f"[INFO] Using fallback GT file: {matches[0].name}")
    return pd.read_csv(matches[0])


def plot_gt_trajectory(df: pd.DataFrame, lighting: str, out_dir: Path):
    x_col, y_col = find_xy_columns(df)

    fig, ax = plt.subplots(figsize=(6.2, 5.0))

    ax.plot(df[x_col], df[y_col], linewidth=2.2, label="Ground Truth")
    ax.scatter(df[x_col].iloc[0], df[y_col].iloc[0], marker="o", s=45, label="Start")
    ax.scatter(df[x_col].iloc[-1], df[y_col].iloc[-1], marker="x", s=55, label="End")

    ax.set_title(f"Ground-Truth Trajectory: {lighting.capitalize()} {TRIAL}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(frameon=True)

    fig.tight_layout()

    out_png = out_dir / f"fig_4_0_gt_trajectory_{lighting}_{TRIAL}.png"
    out_pdf = out_dir / f"fig_4_0_gt_trajectory_{lighting}_{TRIAL}.pdf"

    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    fig.savefig(out_pdf, bbox_inches="tight")
    plt.close(fig)

    print(f"[SAVED] {out_png}")
    print(f"[SAVED] {out_pdf}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--light",
        choices=LIGHTING_ORDER + ["all"],
        default="all",
        help="Lighting condition to plot. Default: all",
    )
    args = parser.parse_args()

    script_path = Path(__file__).resolve()
    base_dir = script_path.parents[1]

    out_dir = base_dir / "thesis_chapter_4_images"
    out_dir.mkdir(parents=True, exist_ok=True)

    lights = LIGHTING_ORDER if args.light == "all" else [args.light]

    for lighting in lights:
        df = load_gt_csv(base_dir, lighting, TRIAL)
        plot_gt_trajectory(df, lighting, out_dir)


if __name__ == "__main__":
    main()
