import matplotlib.pyplot as plt
import numpy as np

# ==========================================
# 0. 이동 평균 함수
# ==========================================
def moving_average(data, window_size):
    if len(data) < window_size:
        return data
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# ==========================================
# 1. 파일 경로 설정
# ==========================================
file_path = "/home/cdsl/libfranka/a2_data/sim6/quintic/qp8.txt_timing.txt" 

# ==========================================
# 2. 데이터 로드 및 통계 계산
# ==========================================
data = {
    "Model": [], "Dynamics": [], "Filter": [], 
    "Trajectory": [], "Control(QP)": [], "Total": []
}

try:
    with open(file_path, "r", encoding="utf-8") as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split()
            if len(parts) < 6: continue
            
            data["Model"].append(int(parts[0]))
            data["Dynamics"].append(int(parts[1]))
            data["Filter"].append(int(parts[2]))
            data["Trajectory"].append(int(parts[3]))
            data["Control(QP)"].append(int(parts[4]))
            data["Total"].append(int(parts[5]))

    # 테이블 데이터 계산
    table_keys = ["Model", "Dynamics", "Filter", "Trajectory", "Control(QP)", "Total"]
    col_labels = ["Avg (us)", "Min (us)", "Max (us)"]
    cell_text = []

    for key in table_keys:
        vals = np.array(data[key])
        if len(vals) == 0: vals = [0]

        avg_val = np.mean(vals)
        min_val = np.min(vals)
        max_val = np.max(vals)
        perc = (avg_val / 1000.0) * 100.0

        cell_text.append([
            f"{avg_val:.2f}", f"{min_val}", f"{max_val}"
        ])

    # ==========================================
    # 3. 시각화 설정
    # ==========================================
    # 색상 팔레트 (Total은 보라색)
    colors = ['#5DA5DA', '#FAA43A', '#60BD68', '#F17CB0', '#B276B2']
    
    # 그래프를 그릴 키 목록
    plot_keys = ["Model", "Dynamics", "Trajectory", "Control(QP)", "Total"]
    
    # 타이틀 목록
    titles = [
        "1. Model Update Time",
        "2. Dynamics Calculation Time",
        "3. Trajectory Generation Time",
        "4. QP Control Time",
        "5. Total Loop Time"
    ]

    # 6행 1열 (그래프 5개 + 테이블 1개)
    fig, axes = plt.subplots(6, 1, figsize=(10, 20))

    WINDOW_SIZE = 100 
    
    # ==========================================
    # 4. 그래프 그리기 (상단 5개)
    # ==========================================
    for i, key in enumerate(plot_keys):
        ax = axes[i]
        raw_data = data[key]
        
        # 스무딩 적용
        smoothed_data = moving_average(raw_data, WINDOW_SIZE)
        steps = range(len(smoothed_data))
        
        # Plot 스타일
        ax.plot(steps, smoothed_data, 
                color=colors[i], 
                linewidth=1.0,      
                marker='o',         
                markersize=4,       
                markevery=150,      
                alpha=0.9,          
                label=key)

        # 디자인 설정
        ax.set_title(titles[i], loc='left', fontsize=14, fontweight='normal')
        ax.set_ylabel("Time [us]", fontsize=11, fontweight='normal')
        
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.legend(loc='upper right', frameon=True)
        
        # 마지막 그래프(4번 인덱스)에만 X축 라벨 표시
        if i < 4:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel("Control Loop Step (Smoothed)", fontsize=11, fontweight='normal')

        # [수정됨] Total 그래프(key == "Total")일 때 1ms 선을 그리지 않음
        # ax.axhline(...) 코드를 삭제했으므로, 
        # matplotlib이 데이터 범위에 맞춰 자동으로 Y축을 확대해 줍니다.

    # ==========================================
    # 5. 테이블 그리기 (하단 1개 - 인덱스 5)
    # ==========================================
    ax_table = axes[5]
    ax_table.axis('off') 
    
    the_table = ax_table.table(
        cellText=cell_text,
        rowLabels=table_keys,
        colLabels=col_labels,
        loc='center',
        cellLoc='center'
    )
    
    the_table.auto_set_font_size(False)
    the_table.set_fontsize(11)
    the_table.scale(1, 2.5) 

    for (row, col), cell in the_table.get_celld().items():
        cell.set_text_props(fontweight='normal')

    plt.tight_layout()
    plt.subplots_adjust(hspace=0.3, bottom=0.05) 

    plt.show()

except FileNotFoundError:
    print(f"오류: '{file_path}' 파일을 찾을 수 없습니다.")
except Exception as e:
    import traceback
    traceback.print_exc()