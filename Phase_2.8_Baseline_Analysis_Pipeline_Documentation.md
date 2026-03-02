# Baseline Analysis Pipeline Documentation (Wheel vs EKF)

## Scope

This documents the offline analysis pipeline created for the EKF-SLAM thesis baseline comparison.  
It covers environment setup, scripts, execution steps, and debugging/fixes encountered while extracting and labeling data.

## 1) Why a Separate Python Environment Was Needed

### Goal

Run offline analysis (CSV processing, plotting, metrics) without breaking the ROS2 Python environment.

### Problem

Installing analysis libraries (`pandas`, `numpy`, `matplotlib`, `scipy`) into system Python can:

- conflict with ROS2 Python dependencies
- break `rclpy` imports or ROS tooling
- create version mismatch issues

### Solution

Use a dedicated virtual environment for analysis only.

## 2) Virtual Environment Setup and Usage

### Create venv (one-time)

From `~/ros2_ws`:

python3 -m venv analysis_venv

## Activate venv (when running analysis scripts)

source analysis_env/bin/activate

## Install analysis-only dependencies

pip install --upgrade pip
pip install pandas numpy matplotlib scipy

## Deactivate venv (when switching back to ROS2 environment)

deactivate

### Rule Used

- **Activate venv** → when running CSV/plot/metrics scripts.  
- **Deactivate venv** → when running ROS2 playback/echo/debug commands (if needed).  

Keep robotics runtime and analysis runtime separated.

## 3) Folder Structure Added for Analysis

Inside `ros2_ws`:
ros_ws/
├── README.md
├── analysis/
    └── scripts/
        └── results/
 
### Where:

- `analysis/scripts/` holds reproducible scripts.  
- `scripts/results/` stores generated CSVs and plots.  
- `ros_ws/README.md` tracks pipeline usage and experiment notes.

## 4) CSV Extraction Outputs

We extracted estimator outputs into CSV:

- **Wheel baseline CSV**  
  `analysis/results/wheel_baseline.csv`

- **EKF baseline CSV**  
  `analysis/results/ekf_baseline.csv`
  
### Columns used:

- `timestamp`, `x`, `y`, `yaw`, `omega_z`, `phase`

### Notes:

- `phase` initially existed but was empty in estimator CSVs.  
- Timestamps were bag-recorded (nanoseconds), later normalized per run for plotting.

## 5) Sanity Checks Performed

We verified:

- **Row count**  
  ```bash
  wc -l analysis/results/wheel_baseline.csv
  wc -l analysis/results/ekf_baseline.csv

## Duration Check

python3 - <<'PY'
import pandas as pd
for name in ["wheel_baseline","ekf_baseline"]:
    df = pd.read_csv(f"analysis/results/{name}.csv")
    print("\n", name, "rows:", len(df))
    print("t_min:", df["timestamp"].min(), "t_max:", df["timestamp"].max(),
          "duration_s:", (df["timestamp"].max()-df["timestamp"].min())*1e-9)
PY

### Result:

- Both runs were ~272 seconds.  
- Absolute timestamps differed across runs (expected).

## 6) Phase Labeling (Key Problem + Fixes)

### Objective

Label every estimator sample with its phase (0/1/2/3/4…) using `/traj_phase`.

### Initial Issue #1: Echo file had no timestamps

The phase echo output looked like:

data: 1
---
data: 2
---
...

So timestamp-based parsing from echo text failed.

### Fix

- Extract phase directly from bag using `rosbag2_py` recorded timestamps (`t` from `read_next()`).

This produced:

- `analysis/results/wheel_phase_timeline.csv`
- `analysis/results/ekf_phase_timeline.csv`

**Format:**
timestamp, phase, t_rel

### Initial Issue #2: All labels became unknown

Even after we had a phase timeline, merge produced `unknown` everywhere.

**Root cause:**

- Phase stream and odom stream did not start at the same time.  
- Normalizing each topic separately (`t_rel`) misaligned timelines.  
- Merge tolerance missed matches.

### Fix

- Use absolute timestamp based labeling, then normalize afterward for plotting.

### Initial Issue #3: `phase_x`, `phase_y` confusion

- Estimator CSV already had a blank `phase` column, so merge created:
  - `phase_x` (empty from estimator)
  - `phase_y` (valid from phase timeline)
- Then code mistakenly filled the wrong column.

### Fix

- Drop the estimator’s empty `phase` column before merge.

### Final Fix (Best Approach): Interval/Step-function Labeling

Even after merge fixes, many samples were still `unknown` because `/traj_phase` may be sparse (e.g., published only at transitions).

**Final solution:**

- Use `merge_asof` with `direction="backward"` and **NO tolerance**  
- Each odom sample gets the most recent phase value (step function)  
- Only samples before first phase publish remain `unknown`

**Final output files:**

- `analysis/results/wheel_labeled_final.csv`
- `analysis/results/ekf_labeled_final.csv`

This produced near-complete labeling:

- Wheel had a small startup unknown fraction  
- EKF had 0 unknown

## 7) Normalizing Time for Plotting

We created per-run relative time:

t_rel = (timestamp - first_timestamp) in seconds

Script: normalize_time.py

Outputs:

analysis/results/wheel_labeled_final_norm.csv

analysis/results/ekf_labeled_final_norm.csv

This step is used only for:

time-series plots

overlay visualization

We intentionally keep absolute timestamps in the “final” labeled CSVs for reproducibility.

## 8) Metric Computation Script

**Script:** `compute_metrics_phasewise.py`

**Output:**

- `analysis/results/metrics_phasewise.csv`

**Workflow:**

- Removes `unknown` phase samples  
- Computes metrics for:
  - ALL samples combined
  - Each phase separately (`groupby("phase")`)

**Note:** No analysis conclusions are documented here (only pipeline).

## 9) Plot Generation Script

**Script:** `plot_baselines.py`

**Inputs:**

- Normalized labeled CSVs (`*_final_norm.csv`)

**Outputs saved under:**

- `analysis/results/trajectory_compare.png`
- `analysis/results/yaw_compare.png`
- `analysis/results/omega_compare.png`

## 10) Improvement Table Script

**Goal:**  
Generate a table comparing wheel vs EKF per phase.

**Issue encountered:**

- Wheel phase values appeared as floats (`"2.0"`)  
- EKF phase values appeared as ints (`"2"`)  
- Index matching failed, resulting in only `ALL` row output

**Fix:**

- Normalize phase labels:  
  `"2.0"` → `"2"`  
- Keep `"ALL"` as `"ALL"`

**Output:**

- `analysis/results/improvement_table.csv`

# Execution Workflow Summary

When in analysis mode:

```bash
source analysis_env/bin/activate
python3 <analysis script>

When switching back to ROS2 tools:

```bash
deactivate
ros2 ...
















