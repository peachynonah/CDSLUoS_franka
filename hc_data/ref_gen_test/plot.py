import pandas as pd
import matplotlib.pyplot as plt

# CSV 로드
csv_path = "refgentest_2.txt"  # 파일 경로 수정
df = pd.read_csv(csv_path, delim_whitespace=True)

# 시간축 (없으면 index 사용)
t = df.index

# 데이터 묶기
pos = ["pos_des_x", "pos_des_y", "pos_des_z"]
vel = ["vel_des_x", "vel_des_y", "vel_des_z"]
acc = ["acc_des_x", "acc_des_y", "acc_des_z"]
force = ["F_FL_x", "F_FL_y", "F_FL_z"]

rows = [pos, vel, acc]
row_names = ["Position", "Velocity", "Acceleration"]

# Figure 생성
fig, axes = plt.subplots(3, 4, figsize=(16, 9), sharex=True)

for i, (row, name) in enumerate(zip(rows, row_names)):
    # x, y, z
    for j in range(3):
        axes[i, j].plot(t, df[row[j]])
        axes[i, j].set_title(f"{name} {['x','y','z'][j]}")
        axes[i, j].grid(True)

    # Force column
    axes[i, 3].plot(t, df[force[i]])
    axes[i, 3].set_title(f"F_FL {['x','y','z'][i]}")
    axes[i, 3].grid(True)

# 공통 설정
for ax in axes[-1, :]:
    ax.set_xlabel("Time step")

plt.tight_layout()
plt.show()
