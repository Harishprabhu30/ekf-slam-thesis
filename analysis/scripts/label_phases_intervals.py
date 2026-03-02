import pandas as pd
import os

def label_intervals(est_csv, phase_csv, out_csv):
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)

    est = pd.read_csv(est_csv).sort_values("timestamp").reset_index(drop=True)
    ph  = pd.read_csv(phase_csv).sort_values("timestamp").reset_index(drop=True)

    # Drop any estimator 'phase' column to avoid collisions
    if "phase" in est.columns:
        est = est.drop(columns=["phase"])

    # Ensure phase column exists in ph
    if "phase" not in ph.columns:
        if "data" in ph.columns:
            ph = ph.rename(columns={"data": "phase"})
        else:
            candidates = [c for c in ph.columns if c not in ["timestamp", "t_rel"]]
            if not candidates:
                raise RuntimeError(f"No phase-like column in {phase_csv}. Columns={list(ph.columns)}")
            ph = ph.rename(columns={candidates[-1]: "phase"})

    # This is the key: no tolerance, pure step-function labeling
    labeled = pd.merge_asof(
        est,
        ph[["timestamp", "phase"]],
        on="timestamp",
        direction="backward"
    )

    labeled["phase"] = labeled["phase"].fillna("unknown")
    labeled.to_csv(out_csv, index=False)

    print("Saved:", out_csv)
    print("Phase counts:\n", labeled["phase"].value_counts())

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 4:
        print("Usage: python3 label_phases_intervals.py <est_csv> <phase_csv> <out_csv>")
        raise SystemExit(1)

    label_intervals(sys.argv[1], sys.argv[2], sys.argv[3])
