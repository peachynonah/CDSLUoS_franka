import numpy as np
import matplotlib.pyplot as plt

EXPECTED_COLS = 40  # 16 + 6 + 6 + 3 + 6 + 3

rows = []
bad_lines = []

with open("test.txt", "r") as f:
    for lineno, line in enumerate(f, start=1):
        parts = line.strip().split()
        if not parts:
            continue
        if len(parts) != EXPECTED_COLS:
            bad_lines.append((lineno, len(parts)))
            continue
        try:
            rows.append([float(x) for x in parts])
        except ValueError:
            bad_lines.append((lineno, "non-float"))
            continue

data = np.array(rows, dtype=float)

if data.ndim != 2 or data.shape[1] != EXPECTED_COLS:
    raise RuntimeError(f"Parsed data shape is {data.shape}, expected (*,{EXPECTED_COLS}). "
                       f"Bad lines example: {bad_lines[:5]}")

print(f"Loaded {data.shape[0]} rows with {data.shape[1]} cols.")
print(f"Skipped {len(bad_lines)} bad lines. Example:", bad_lines[:5])

# ----------------------------
# Indices based on your myfile layout
# O_T_EE (0..15): using translation at [12,13,14] (your assumption)
px = data[:, 12]
py = data[:, 13]
pz = data[:, 14]

# force_FL (16..21)
Fx = data[:, 16]
Fy = data[:, 17]
Fz = data[:, 18]

# desired position (28..30)
px_d = data[:, 28]
py_d = data[:, 29]
pz_d = data[:, 30]

# measured velocity (31..36): [vx, vy, vz, wx, wy, wz]
vx = data[:, 31]
vy = data[:, 32]
vz = data[:, 33]

# desired velocity (37..39): [vx_d, vy_d, vz_d]
vx_d = data[:, 37]
vy_d = data[:, 38]
vz_d = data[:, 39]
# ----------------------------

fig, axs = plt.subplots(3, 3, figsize=(14, 10), sharex=True)

# (row 1, col 1): x position vs desired x
axs[0, 0].plot(px, label="x")
axs[0, 0].plot(px_d, "--", label="x_d")
axs[0, 0].set_title("EE x position")

# (row 2, col 1): y position vs desired y
axs[1, 0].plot(py, label="y")
axs[1, 0].plot(py_d, "--", label="y_d")
axs[1, 0].set_title("EE y position")

# (row 3, col 1): z position vs desired z
axs[2, 0].plot(pz, label="z")
axs[2, 0].plot(pz_d, "--", label="z_d")
axs[2, 0].set_title("EE z position")

# (row 1, col 2): x velocity vs desired x velocity
axs[0, 1].plot(vx, label="vx")
axs[0, 1].plot(vx_d, "--", label="vx_d")
axs[0, 1].set_title("EE x velocity")

# (row 2, col 2): y velocity vs desired y velocity
axs[1, 1].plot(vy, label="vy")
axs[1, 1].plot(vy_d, "--", label="vy_d")
axs[1, 1].set_title("EE y velocity")

# (row 3, col 2): z velocity vs desired z velocity
axs[2, 1].plot(vz, label="vz")
axs[2, 1].plot(vz_d, "--", label="vz_d")
axs[2, 1].set_title("EE z velocity")

# (row 1, col 3): force_FL x input
axs[0, 2].plot(Fx, label="Fx")
axs[0, 2].set_title("force_FL: Fx")

# (row 2, col 3): force_FL y input
axs[1, 2].plot(Fy, label="Fy")
axs[1, 2].set_title("force_FL: Fy")

# (row 3, col 3): force_FL z input  (assuming your last line was a typo)
axs[2, 2].plot(Fz, label="Fz")
axs[2, 2].set_title("force_FL: Fz")

# cosmetics
for ax in axs.ravel():
    ax.grid(True)
    ax.legend()

axs[2, 0].set_xlabel("sample")
axs[2, 1].set_xlabel("sample")
axs[2, 2].set_xlabel("sample")

plt.tight_layout()
plt.show()
