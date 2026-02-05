import pandas as pd
import matplotlib.pyplot as plt

# ===== load =====
csv_path = "oritest_t6.txt"
df = pd.read_csv(csv_path, delim_whitespace=True)

t = df.index  # time-step as x-axis

# ===== column groups =====
des_pos = ["pos_des_x", "pos_des_y", "pos_des_z"]
des_vel = ["vel_des_x", "vel_des_y", "vel_des_z"]
des_acc = ["acc_des_x", "acc_des_y", "acc_des_z"]

cur_pos = ["pos_x", "pos_y", "pos_z"]
cur_vel = ["vel_x", "vel_y", "vel_z"]

# forces
force_lin = ["F_FL_x", "F_FL_y", "F_FL_z"]
force_rot = ["F_FL_rx", "F_FL_ry", "F_FL_rz"]

# errors
rot_err = ["error_rx", "error_ry", "error_rz"]

rows = [
    ("Position", des_pos, cur_pos),
    ("Velocity", des_vel, cur_vel),
    ("Acceleration", des_acc, None),
]

# ===== figure =====
fig, axes = plt.subplots(3, 6, figsize=(22, 9), sharex=True)

xyz = ["x", "y", "z"]

for i, (name, des_cols, cur_cols) in enumerate(rows):
    # 1~3열: x, y, z
    for j in range(3):
        axes[i, j].plot(t, df[des_cols[j]], label="desired")
        if cur_cols is not None:
            axes[i, j].plot(t, df[cur_cols[j]], label="current")
            axes[i, j].legend()

        axes[i, j].set_title(f"{name} {xyz[j]}")
        axes[i, j].grid(True)

    # 4열: linear force
    axes[i, 3].plot(t, df[force_lin[i]])
    axes[i, 3].set_title(f"F_FL {xyz[i]}")
    axes[i, 3].grid(True)

    # 5열: rotation error
    axes[i, 4].plot(t, df[rot_err[i]])
    axes[i, 4].set_title(f"error_r{xyz[i]}")
    axes[i, 4].grid(True)

    # 6열: rotational force
    axes[i, 5].plot(t, df[force_rot[i]])
    axes[i, 5].set_title(f"F_FL_r{xyz[i]}")
    axes[i, 5].grid(True)

# x-axis label
for ax in axes[-1, :]:
    ax.set_xlabel("Time step")

plt.tight_layout()
plt.show()
