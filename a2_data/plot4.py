import matplotlib.pyplot as plt
import numpy as np
import os

# ==========================================
# 1. 파일 경로 설정
# ==========================================
# C++ 코드 실행 시 사용한 출력 파일 경로로 수정하세요.
file_path = "/home/cdsl/libfranka/a2_data/sim6/quintic/pd6.txt"

# ==========================================
# 2. 데이터 로드 및 파싱
# ==========================================
t_data = []
pos_act = [[], [], []]   # x, y, z
pos_des = [[], [], []]
vel_act = [[], [], []]   # vx, vy, vz
ori_act = [[], [], []]   # qx, qy, qz
ori_des = [[], [], []]
tau_data = [[] for _ in range(7)] # Joint 1 ~ 7 Torque

try:
    with open(file_path, "r", encoding="utf-8") as f:
        # 헤더가 있다면 읽고 넘어갑니다. (없으면 주석 처리)
        header = f.readline() 
        
        for line in f:
            parts = line.strip().split()
            
            # 데이터 개수 체크: 총 23개 열이 있어야 함
            # Time(1) + Pos(6) + Vel(3) + Ori(6) + Tau(7) = 23
            if len(parts) < 23:
                continue
            
            # 1. Time
            t_data.append(float(parts[0]))
            
            # 2. Position (Actual: 1~3, Desired: 4~6)
            for i in range(3):
                pos_act[i].append(float(parts[1+i]))
                pos_des[i].append(float(parts[4+i]))
            
            # 3. Velocity (Actual: 7~9)
            for i in range(3):
                vel_act[i].append(float(parts[7+i]))
                
            # 4. Orientation (Actual: 10~12, Desired: 13~15)
            # 쿼터니언의 x, y, z 성분
            for i in range(3):
                ori_act[i].append(float(parts[10+i]))
                ori_des[i].append(float(parts[13+i]))
                
            # 5. Torque (16~22) -> 7 Joints
            for i in range(7):
                tau_data[i].append(float(parts[16+i]))

    print(f"✅ 데이터 로드 완료: {len(t_data)} steps")

    # ==========================================
    # 3. 그래프 그리기 (4행 1열)
    # ==========================================
    fig, axes = plt.subplots(4, 1, figsize=(12, 16), sharex=True)
    
    # 공통 스타일 설정
    lw_act = 1.0       # 실제값 선 두께
    lw_des = 1.2       # 목표값 선 두께
    ls_des = '--'      # 목표값 점선 스타일
    xyz_labels = ['x', 'y', 'z']
    colors = ['r', 'g', 'b'] # x=Red, y=Green, z=Blue

    # --- Plot 1: Cartesian Position ---
    ax = axes[0]
    for i in range(3):
        ax.plot(t_data, pos_act[i], color=colors[i], linewidth=lw_act, label=f'Act {xyz_labels[i]}')
        ax.plot(t_data, pos_des[i], color=colors[i], linestyle=ls_des, linewidth=lw_des, alpha=0.7)
    ax.set_ylabel("Position [m]")
    ax.set_title("1. Cartesian Position (Actual vs Desired)", fontweight='normal', loc='left')
    ax.legend(loc='upper right', ncol=3)
    ax.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 2: Cartesian Velocity ---
    ax = axes[1]
    for i in range(3):
        ax.plot(t_data, vel_act[i], color=colors[i], linewidth=lw_act, label=f'Vel {xyz_labels[i]}')
    ax.set_ylabel("Velocity [m/s]")
    ax.set_title("2. Cartesian Velocity (Actual)", fontweight='normal', loc='left')
    ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 3: Orientation (Quaternion xyz) ---
    ax = axes[2]
    for i in range(3):
        ax.plot(t_data, ori_act[i], color=colors[i], linewidth=lw_act, label=f'q_{xyz_labels[i]}')
        ax.plot(t_data, ori_des[i], color=colors[i], linestyle=ls_des, linewidth=lw_des, alpha=0.7)
    ax.set_ylabel("Quaternion (x,y,z)")
    ax.set_title("3. Orientation (Actual vs Desired)", fontweight='normal', loc='left')
    ax.legend(loc='upper right', ncol=3)
    ax.grid(True, linestyle='--', alpha=0.6)

    # --- Plot 4: Torque (Joint 1~7) ---
    ax = axes[3]
    # 7개 관절을 구별하기 위한 색상맵 (무지개색)
    tau_colors = plt.cm.jet(np.linspace(0, 1, 7)) 
    
    for i in range(7):
        ax.plot(t_data, tau_data[i], color=tau_colors[i], linewidth=1.0, label=f'J{i+1}')
    
    ax.set_ylabel("Torque [Nm]")
    ax.set_xlabel("Time [s]")
    ax.set_title("4. Joint Torques (1-7)", fontweight='normal', loc='left')
    # 범례를 그래프 밖이나 아래쪽이 아닌, 그래프 안쪽 상단에 가로로 배치
    ax.legend(loc='upper right', ncol=7, fontsize='small', framealpha=0.5)
    ax.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()

except FileNotFoundError:
    print(f"❌ 파일을 찾을 수 없습니다: {file_path}")
except Exception as e:
    import traceback
    traceback.print_exc()