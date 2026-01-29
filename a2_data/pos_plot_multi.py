import matplotlib.pyplot as plt
import os

# ==========================================
# 1. 설정 영역
# ==========================================

# 데이터 파일들이 있는 기본 폴더 경로
base_directory = "/home/cdsl/libfranka/a2_data/sim6/quintic/"

# Plot하고 싶은 파일명들을 리스트에 추가하세요 (원하는 만큼 추가 가능)
file_names = [
    "qp8.txt",
    "qp9.txt",
    "qp10.txt", 
    "qp11.txt", 
    # "qp5.txt",     
    # "qp6.txt",     
]

# ==========================================

# 그래프 초기화 (3행 1열)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# 파일 리스트를 순회하며 데이터 읽기 및 그리기
for file_name in file_names:
    file_path = os.path.join(base_directory, file_name)
    
    # 데이터를 저장할 리스트 초기화 (파일마다 새로 비움)
    t_data = []
    x_data, y_data, z_data = [], [], []
    xd_data, yd_data, zd_data = [], [], []

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            header = f.readline() # 헤더 건너뛰기
            
            for line in f:
                parts = line.strip().split()
                
                # 데이터 유효성 검사
                if len(parts) < 7:
                    continue
                
                # 데이터 파싱
                t_data.append(float(parts[0]))
                
                x_data.append(float(parts[1]))
                y_data.append(float(parts[2]))
                z_data.append(float(parts[3]))
                
                xd_data.append(float(parts[4]))
                yd_data.append(float(parts[5]))
                zd_data.append(float(parts[6]))

        # --- 그래프 그리기 (겹쳐 그리기) ---
        # 파일 이름(확장자 제외)을 라벨로 사용
        label_name = os.path.splitext(file_name)[0]

        # 1. X축 Plot
        # p는 plot 객체를 반환받아 색상을 통일하기 위함
        p = ax1.plot(t_data, x_data, '-', label=f'{label_name} Act', linewidth=1.5)
        color = p[0].get_color() # Actual 선의 색상을 가져옴
        ax1.plot(t_data, xd_data, '--', color=color, label=f'{label_name} Des', linewidth=1.0, alpha=0.7)

        # 2. Y축 Plot
        ax2.plot(t_data, y_data, '-', color=color, label=f'{label_name} Act', linewidth=1.5)
        ax2.plot(t_data, yd_data, '--', color=color, label=f'{label_name} Des', linewidth=1.0, alpha=0.7)

        # 3. Z축 Plot
        ax3.plot(t_data, z_data, '-', color=color, label=f'{label_name} Act', linewidth=1.5)
        ax3.plot(t_data, zd_data, '--', color=color, label=f'{label_name} Des', linewidth=1.0, alpha=0.7)

        print(f"[성공] {file_name} 로드 완료")

    except FileNotFoundError:
        print(f"[오류] '{file_path}' 파일을 찾을 수 없습니다. 건너뜁니다.")
    except Exception as e:
        print(f"[오류] {file_name} 처리 중 문제 발생: {e}")

# ==========================================
# 그래프 스타일 꾸미기
# ==========================================

# 첫 번째 subplot (X)
ax1.set_ylabel("x [m]")
ax1.set_title("Cartesian Position Comparison")
ax1.grid(True)
ax1.legend(loc='upper right', fontsize='small', ncol=len(file_names)) 

# 두 번째 subplot (Y)
ax2.set_ylabel("y [m]")
ax2.grid(True)
# ax2.legend() # 범례가 너무 많으면 생략 가능

# 세 번째 subplot (Z)
ax3.set_ylabel("z [m]")
ax3.set_xlabel("Time [s]")
ax3.grid(True)
# ax3.legend()

plt.tight_layout()
plt.show()