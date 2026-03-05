#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt


def load_data(filepath: str):
    """
    Robust loader for whitespace separated log files.
    Lines with wrong column counts are skipped automatically.
    """
    with open(filepath, "r") as f:
        header = f.readline().strip()
        if not header:
            raise ValueError("Header line is empty.")

        cols = header.split()
        ncols = len(cols)

        rows = []
        bad_lines = 0

        for line_no, line in enumerate(f, start=2):
            s = line.strip()
            if not s:
                continue

            parts = s.split()
            if len(parts) != ncols:
                bad_lines += 1
                continue

            try:
                row = [float(x) for x in parts]
                rows.append(row)
            except ValueError:
                bad_lines += 1
                continue

    if len(rows) == 0:
        raise ValueError("No valid data rows found.")

    data = np.array(rows)
    t = np.arange(data.shape[0])
    col_idx = {name: i for i, name in enumerate(cols)}

    if bad_lines > 0:
        print(f"[INFO] skipped {bad_lines} corrupted lines")

    return t, data, col_idx


def get_col(data, col_idx, name):
    if name not in col_idx:
        # 디버깅 편하게: 가장 비슷한 후보 몇 개 보여주기
        candidates = [k for k in col_idx.keys() if name.lower() in k.lower() or k.lower() in name.lower()]
        msg = f"Column '{name}' not found."
        if candidates:
            msg += f" Candidates: {candidates[:10]}"
        raise KeyError(msg)
    return data[:, col_idx[name]]


def plot_pair(ax, t, y1, y2, label1, label2, title):
    ax.plot(t, y1, label=label1)
    ax.plot(t, y2, label=label2)
    ax.set_title(title)
    ax.set_xlabel("time (samples)")
    ax.grid(True)
    ax.legend(fontsize=8)


def plot_single(ax, t, y, label, title):
    ax.plot(t, y, label=label)
    ax.set_title(title)
    ax.set_xlabel("time (samples)")
    ax.grid(True)
    ax.legend(fontsize=8)


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 plot.py dataname.txt")
        sys.exit(1)

    filepath = sys.argv[1]
    t, data, col_idx = load_data(filepath)

    # ======================================================
    # Figure 1 : 3x3
    # - Position (x,y,z)
    # - Velocity (x,y,z)
    # - Force: F_FL vs F_FL_ana (x,y,z)
    # ======================================================

    fig1, axs = plt.subplots(3, 3, figsize=(14, 9), sharex=True)
    fig1.suptitle("Position / Velocity / Force Tracking", fontsize=14)

    # Position (desired vs actual)
    plot_pair(axs[0, 0], t,
              get_col(data, col_idx, "pos_des_x"),
              get_col(data, col_idx, "pos_x"),
              "pos_des_x", "pos_x", "Position X")

    plot_pair(axs[1, 0], t,
              get_col(data, col_idx, "pos_des_y"),
              get_col(data, col_idx, "pos_y"),
              "pos_des_y", "pos_y", "Position Y")

    plot_pair(axs[2, 0], t,
              get_col(data, col_idx, "pos_des_z"),
              get_col(data, col_idx, "pos_z"),
              "pos_des_z", "pos_z", "Position Z")

    # Velocity (desired vs actual, linear)
    plot_pair(axs[0, 1], t,
              get_col(data, col_idx, "vel_des_x"),
              get_col(data, col_idx, "vel_x"),
              "vel_des_x", "vel_x", "Velocity X")

    plot_pair(axs[1, 1], t,
              get_col(data, col_idx, "vel_des_y"),
              get_col(data, col_idx, "vel_y"),
              "vel_des_y", "vel_y", "Velocity Y")

    plot_pair(axs[2, 1], t,
              get_col(data, col_idx, "vel_des_z"),
              get_col(data, col_idx, "vel_z"),
              "vel_des_z", "vel_z", "Velocity Z")

    # Force (F_FL vs F_FL_ana, translational)
    plot_pair(axs[0, 2], t,
              get_col(data, col_idx, "F_FL_x"),
              get_col(data, col_idx, "F_FL_ana_x"),
              "F_FL_x", "F_FL_ana_x", "Force X")

    plot_pair(axs[1, 2], t,
              get_col(data, col_idx, "F_FL_y"),
              get_col(data, col_idx, "F_FL_ana_y"),
              "F_FL_y", "F_FL_ana_y", "Force Y")

    plot_pair(axs[2, 2], t,
              get_col(data, col_idx, "F_FL_z"),
              get_col(data, col_idx, "F_FL_ana_z"),
              "F_FL_z", "F_FL_ana_z", "Force Z")

    fig1.tight_layout(rect=[0, 0, 1, 0.96])

    # ======================================================
    # Figure 2 : 3x2
    # - Body RPY: desired vs actual (R,P,Y)
    # - Rotational force: F_FL vs F_FL_ana (rx,ry,rz)
    # ======================================================

    fig2, axs = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    fig2.suptitle("Body RPY Desired vs Actual and Rotational Forces", fontsize=14)

    # RPY (desired vs actual)
    plot_pair(axs[0, 0], t,
              get_col(data, col_idx, "bodyRPY_des_R"),
              get_col(data, col_idx, "bodyRPY_R"),
              "bodyRPY_des_R", "bodyRPY_R", "Roll (desired vs actual)")

    plot_pair(axs[1, 0], t,
              get_col(data, col_idx, "bodyRPY_des_P"),
              get_col(data, col_idx, "bodyRPY_P"),
              "bodyRPY_des_P", "bodyRPY_P", "Pitch (desired vs actual)")

    plot_pair(axs[2, 0], t,
              get_col(data, col_idx, "bodyRPY_des_Y"),
              get_col(data, col_idx, "bodyRPY_Y"),
              "bodyRPY_des_Y", "bodyRPY_Y", "Yaw (desired vs actual)")

    # Rotational forces (F_FL vs F_FL_ana)
    plot_pair(axs[0, 1], t,
              get_col(data, col_idx, "F_FL_rx"),
              get_col(data, col_idx, "F_FL_ana_rx"),
              "F_FL_rx", "F_FL_ana_rx", "Force rx")

    plot_pair(axs[1, 1], t,
              get_col(data, col_idx, "F_FL_ry"),
              get_col(data, col_idx, "F_FL_ana_ry"),
              "F_FL_ry", "F_FL_ana_ry", "Force ry")

    plot_pair(axs[2, 1], t,
              get_col(data, col_idx, "F_FL_rz"),
              get_col(data, col_idx, "F_FL_ana_rz"),
              "F_FL_rz", "F_FL_ana_rz", "Force rz")

    fig2.tight_layout(rect=[0, 0, 1, 0.96])

    
    plt.show()


if __name__ == "__main__":
    main()