import pandas as pd
import matplotlib.pyplot as plt

# ===== load =====
csv_path = "FLtest_t3.txt"  # 같은 폴더면 파일명만 OK
df = pd.read_csv(csv_path, delim_whitespace=True)

t = df.index  # time column 없으니 index를 time-step으로 사용

# ===== column groups =====
des_pos = ["pos_des_x", "pos_des_y", "pos_des_z"]
des_vel = ["vel_des_x", "vel_des_y", "vel_des_z"]
des_acc = ["acc_des_x", "acc_des_y", "acc_des_z"]

cur_pos = ["pos_x", "pos_y", "pos_z"]
cur_vel = ["vel_x", "vel_y", "vel_z"]

force = ["F_FL_x", "F_FL_y", "F_FL_z"]

rows = [
    ("Position", des_pos, cur_pos),   # desired + current
    ("Velocity", des_vel, cur_vel),   # desired + current
    ("Acceleration", des_acc, None),  # desired only
]

# ===== figure =====
fig, axes = plt.subplots(3, 4, figsize=(16, 9), sharex=True)

xyz = ["x", "y", "z"]

for i, (name, des_cols, cur_cols) in enumerate(rows):
    # 1~3열: x,y,z
    for j in range(3):
        # desired
        axes[i, j].plot(t, df[des_cols[j]], label="desired")
        # current (pos/vel만)
        if cur_cols is not None:
            axes[i, j].plot(t, df[cur_cols[j]], label="current")
            axes[i, j].legend()

        axes[i, j].set_title(f"{name} {xyz[j]}")
        axes[i, j].grid(True)

    # 4열: force (행별로 x,y,z)
    axes[i, 3].plot(t, df[force[i]])
    axes[i, 3].set_title(f"F_FL {xyz[i]}")
    axes[i, 3].grid(True)

for ax in axes[-1, :]:
    ax.set_xlabel("Time step")

plt.tight_layout()
plt.show()
