import pandas as pd
import matplotlib.pyplot as plt

# Mapping phase numbers to descriptive names
phase_name = {
    "0": "Stop",
    "1": "Straight",
    "2": "Square",
    "3": "CW Turn",
    "4": "Arc"
}

def add_phase_markers(ax, df, time_col="t_rel", phase_col="phase", alpha=0.2):
    """
    Add vertical lines at phase transitions on a plot with descriptive phase names.
    df: DataFrame with time and phase columns (sorted by time)
    """
    # Keep only known phases
    d = df[[time_col, phase_col]].copy()
    d = d[d[phase_col] != "unknown"]
    d = d.sort_values(time_col)

    # Detect phase changes
    changes = d[phase_col].ne(d[phase_col].shift())
    trans = d[changes]

    # Draw vertical lines and labels
    for _, row in trans.iterrows():
        t = row[time_col]
        ph = str(int(float(row[phase_col])))  # normalize to string key
        label = phase_name.get(ph, f"P{ph}")  # use descriptive name if available
        ax.axvline(t, linestyle="--", linewidth=1, alpha=alpha, color="gray")
        ax.text(t, ax.get_ylim()[1], label, rotation=90,
                va="top", ha="right", alpha=alpha, fontsize=8)

# --- Load labeled data ---
wheel = pd.read_csv("analysis/results/wheel_labeled_final_norm.csv")
ekf   = pd.read_csv("analysis/results/ekf_labeled_final_norm.csv")

# Remove unknown phases
wheel = wheel[wheel["phase"] != "unknown"]
ekf   = ekf[ekf["phase"] != "unknown"]

# --- Trajectory overlay ---
plt.figure(figsize=(6,6))
plt.plot(wheel["x"], wheel["y"], label="Wheel", linewidth=1)
plt.plot(ekf["x"], ekf["y"], label="EKF", linewidth=1)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Trajectory: Wheel vs EKF")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("analysis/results/trajectory_compare.png", dpi=200)

# --- Yaw vs Time ---
fig, ax = plt.subplots(figsize=(8,4))
ax.plot(wheel["t_rel"], wheel["yaw"], label="Wheel", linewidth=1)
ax.plot(ekf["t_rel"], ekf["yaw"], label="EKF", linewidth=1)
add_phase_markers(ax, ekf, time_col="t_rel", phase_col="phase")
ax.set_xlabel("t (s)")
ax.set_ylabel("Yaw (rad)")
ax.set_title("Yaw vs Time")
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.savefig("analysis/results/yaw_compare.png", dpi=200)

# --- Angular Velocity vs Time ---
fig, ax = plt.subplots(figsize=(8,4))
ax.plot(wheel["t_rel"], wheel["omega_z"], label="Wheel", linewidth=1)
ax.plot(ekf["t_rel"], ekf["omega_z"], label="EKF", linewidth=1)
add_phase_markers(ax, ekf, time_col="t_rel", phase_col="phase")
ax.set_xlabel("t (s)")
ax.set_ylabel("omega_z (rad/s)")
ax.set_title("Angular Velocity (omega_z) vs Time")
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.savefig("analysis/results/omega_compare.png", dpi=200)

print("Saved plots in analysis/results/ with phase names")
