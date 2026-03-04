import pandas as pd
import matplotlib.pyplot as plt
import sys

# ── 데이터 로드 ──────────────────────────────────────────────
filename = sys.argv[1] if len(sys.argv) > 1 else "robot_data.txt"
df = pd.read_csv(filename, sep=r'\s+')

dt = 0.001
df['time'] = df.index * dt
t = df['time']

# ── Figure 설정: 3행 4열 ──────────────────────────────────────
fig, axes = plt.subplots(3, 4, figsize=(22, 14))
fig.suptitle('Robot State & Force Comparison', fontsize=16, fontweight='bold')

labels_pos = ['X', 'Y', 'Z']
labels_rot = ['RX', 'RY', 'RZ']

# ════════════════════════════════════════════════════════════
# Col 0: Position  (pos_x/y/z vs pos_des_x/y/z)
# ════════════════════════════════════════════════════════════
cur_pos = ['pos_x',     'pos_y',     'pos_z']
des_pos = ['pos_des_x', 'pos_des_y', 'pos_des_z']
colors  = ['tab:blue', 'tab:orange', 'tab:green']

for row, (lbl, cur, des, c) in enumerate(zip(labels_pos, cur_pos, des_pos, colors)):
    ax = axes[row, 0]
    ax.plot(t, df[cur], color=c,             label='current')
    ax.plot(t, df[des], color=c, linestyle='--', alpha=0.6, label='desired')
    ax.set_title(f'pos_{lbl}')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('position [m]')
    ax.legend()
    ax.grid(True)

# ════════════════════════════════════════════════════════════
# Col 1: Velocity  (vel_x/y/z vs vel_des_x/y/z)
# ════════════════════════════════════════════════════════════
cur_vel = ['vel_x',     'vel_y',     'vel_z']
des_vel = ['vel_des_x', 'vel_des_y', 'vel_des_z']

for row, (lbl, cur, des, c) in enumerate(zip(labels_pos, cur_vel, des_vel, colors)):
    ax = axes[row, 1]
    ax.plot(t, df[cur], color=c,             label='current')
    ax.plot(t, df[des], color=c, linestyle='--', alpha=0.6, label='desired')
    ax.set_title(f'vel_{lbl}')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('velocity [m/s]')
    ax.legend()
    ax.grid(True)

# ════════════════════════════════════════════════════════════
# Col 2: Translational Force  (F_FL_x/y/z vs F_FL_ana_x/y/z)
# ════════════════════════════════════════════════════════════
geo_Ft = ['F_FL_x',     'F_FL_y',     'F_FL_z']
ana_Ft = ['F_FL_ana_x', 'F_FL_ana_y', 'F_FL_ana_z']

for row, (lbl, geo, ana) in enumerate(zip(labels_pos, geo_Ft, ana_Ft)):
    ax = axes[row, 2]
    ax.plot(t, df[geo], color='tab:blue', label='F_FL (geo)')
    ax.plot(t, df[ana], color='tab:red',  label='F_FL_ana', linestyle='--')
    ax.set_title(f'Force trans {lbl}')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('force [N]')
    ax.legend()
    ax.grid(True)

# ════════════════════════════════════════════════════════════
# Col 3: Rotational Force  (F_FL_rx/ry/rz vs F_FL_ana_rx/ry/rz)
# ════════════════════════════════════════════════════════════
geo_Fr = ['F_FL_rx',     'F_FL_ry',     'F_FL_rz']
ana_Fr = ['F_FL_ana_rx', 'F_FL_ana_ry', 'F_FL_ana_rz']

for row, (lbl, geo, ana) in enumerate(zip(labels_rot, geo_Fr, ana_Fr)):
    ax = axes[row, 3]
    ax.plot(t, df[geo], color='tab:blue', label='F_FL (geo)')
    ax.plot(t, df[ana], color='tab:red',  label='F_FL_ana', linestyle='--')
    ax.set_title(f'Force rot {lbl}')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('torque [Nm]')
    ax.legend()
    ax.grid(True)

# ── 저장 & 출력 ───────────────────────────────────────────────
plt.tight_layout()
plt.savefig('robot_plot.png', dpi=150)
plt.show()
print("Saved: robot_plot.png")