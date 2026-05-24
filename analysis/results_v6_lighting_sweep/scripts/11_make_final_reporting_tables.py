from pathlib import Path
import pandas as pd

WS = Path("/media/vgtu/New Volume/Harish_Thesis/ros2_ws")
ROOT = WS / "analysis" / "results_v6_lighting_sweep"

FINAL = ROOT / "final_reporting"
FINAL.mkdir(parents=True, exist_ok=True)

MAIN_TABLE = ROOT / "tables" / "v6_paper_summary_table.csv"
MAIN_FULL = ROOT / "tables" / "v6_mean_std_by_lighting_estimator.csv"

ABL_TABLE = ROOT / "ablations" / "default_vs_tuned_t01" / "tables" / "orbslam3_default_vs_tuned_t01.csv"
ABL_EFFECT = ROOT / "ablations" / "default_vs_tuned_t01" / "tables" / "orbslam3_ablation_effectiveness_t01.csv"

OUT_MAIN = FINAL / "table_1_main_v6_lighting_mean_std.csv"
OUT_ABL = FINAL / "table_2_orbslam3_default_vs_tuned_ablation_t01.csv"
OUT_EFFECT = FINAL / "table_3_orbslam3_ablation_effectiveness_t01.csv"

def main():
    if MAIN_TABLE.exists():
        main_table = pd.read_csv(MAIN_TABLE)
        main_table.to_csv(OUT_MAIN, index=False)
        print(f"Saved: {OUT_MAIN}")
        print("\nMain V6 lighting table:")
        print(main_table.to_string(index=False))
    else:
        print(f"Missing: {MAIN_TABLE}")

    if ABL_TABLE.exists():
        abl = pd.read_csv(ABL_TABLE)
        abl.to_csv(OUT_ABL, index=False)
        print(f"\nSaved: {OUT_ABL}")
        print("\nORB ablation table:")
        print(abl.to_string(index=False))
    else:
        print(f"Missing: {ABL_TABLE}")

    if ABL_EFFECT.exists():
        eff = pd.read_csv(ABL_EFFECT)
        eff.to_csv(OUT_EFFECT, index=False)
        print(f"\nSaved: {OUT_EFFECT}")
        print("\nORB ablation effectiveness table:")
        print(eff.to_string(index=False))
    else:
        print(f"Missing: {ABL_EFFECT}")

    print(f"\nFinal reporting folder: {FINAL}")

if __name__ == "__main__":
    main()
