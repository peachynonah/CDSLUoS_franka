import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# ── Load data ──────────────────────────────────────────────────────────────────
filename = sys.argv[1] if len(sys.argv) > 1 else "data.txt"
if not os.path.exists(filename):
    raise FileNotFoundError(f"Data file '{filename}' not found.")

df = pd.read_csv(filename, sep=r'\s+')
time = range(len(df))

# ── Figure: 3 rows × 2 columns ────────────────────────────────────────────────
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle("Body RPY & Force Analysis", fontsize=14, fontweight="bold")

# ── Column 1: RPY desired vs actual vs error ───────────────────────────────────
rpy_info = [
    ("bodyRPY_des_R", "bodyRPY_R", "error_ana_rx", "Roll"),
    ("bodyRPY_des_P", "bodyRPY_P", "error_ana_ry", "Pitch"),
    ("bodyRPY_des_Y", "bodyRPY_Y", "error_ana_rz", "Yaw"),
]

for row, (col_des, col_act, col_err, title) in enumerate(rpy_info):
    ax = axes[row, 0]
    ax.plot(time, df[col_des], label="desired", color="#3498db", linestyle="--", linewidth=2.0)
    ax.plot(time, df[col_act], label="actual",  color="#e74c3c", linestyle="-.", linewidth=2.0)
    ax.plot(time, df[col_err], label="error",   color="#2ecc71", linewidth=1.5, alpha=0.85)
    ax.set_title(title)
    ax.set_ylabel("rad")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    if row == 2:
        ax.set_xlabel("Sample index")

# ── Column 2: Force rotational components ─────────────────────────────────────
force_info = [
    ("F_FL_ana_rx", "Force rx", "#f39c12"),
    ("F_FL_ana_ry", "Force ry", "#9b59b6"),
    ("F_FL_ana_rz", "Force rz", "#1abc9c"),
]

for row, (col, title, color) in enumerate(force_info):
    ax = axes[row, 1]
    ax.plot(time, df[col], label=col, color=color, linewidth=1.5)
    ax.set_title(title)
    ax.set_ylabel("N·m")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    if row == 2:
        ax.set_xlabel("Sample index")

plt.tight_layout()
plt.savefig("plot.png", dpi=150)
print("Saved plot.png")
plt.show()