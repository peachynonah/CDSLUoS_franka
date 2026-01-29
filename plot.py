import matplotlib.pyplot as plt

# ==========================================
# 0. 그래프 스타일 설정 (글씨 크기 등)
# ==========================================
# 전반적인 폰트 사이즈를 키웁니다.
plt.rcParams.update({
    'font.size': 12,          # 기본 글씨 크기 확대
    'axes.titlesize': 14,     # 제목 크기
    'axes.labelsize': 13,     # 축 라벨 크기
    'xtick.labelsize': 11,    # x축 눈금 크기
    'ytick.labelsize': 11,    # y축 눈금 크기
    'legend.fontsize': 11,    # 범례 크기
    'lines.linewidth': 2.5    # 기본 선 굵기 확대
})

# ==========================================
# 1. 설정 (User Configuration)
# ==========================================
# 비교하고 싶은 두 개의 파일 경로
# file_path1 = "/home/cdsl/libfranka/a2_data/sim1/pd8.txt"      # PD 제어 데이터
# file_path2 = "/home/cdsl/libfranka/a2_data/sim1/qp5.txt"      # QPC 제어 데이터

file_path1 = "/home/cdsl/libfranka/a2_data/sim1/pd8.txt"      # PD 제어 데이터
file_path2 = "/home/cdsl/libfranka/a2_data/sim4/qp12.txt"      # QPC 제어 데이터

# 플롯하고 싶은 최대 시간 (초)
MAX_TIME = 9

# ==========================================
# 2. 데이터 로드 함수 정의
# ==========================================
def load_data(path, max_t=None):
    """
    파일 경로를 입력받아 데이터를 읽되, max_t 시간을 넘어가면 중단합니다.
    """
    t, x, y, z = [], [], [], []
    x_d, y_d, z_d = [], [], []
    
    try:
        with open(path, "r", encoding="utf-8") as f:
            header = f.readline() # 헤더 스킵
            
            for line in f:
                parts = line.strip().split()
                if len(parts) < 7: continue
                
                current_time = float(parts[0])
                if max_t is not None and current_time > max_t:
                    break
                
                t.append(current_time)
                x.append(float(parts[1]))
                y.append(float(parts[2]))
                z.append(float(parts[3]))
                x_d.append(float(parts[4]))
                y_d.append(float(parts[5]))
                z_d.append(float(parts[6]))
                
        return t, x, y, z, x_d, y_d, z_d
    
    except FileNotFoundError:
        print(f"[오류] 파일을 찾을 수 없습니다: {path}")
        return None
    except Exception as e:
        print(f"[오류] 파일 읽기 중 문제 발생 ({path}): {e}")
        return None

# ==========================================
# 3. 데이터 불러오기
# ==========================================
print(f"데이터 로딩 중... (최대 {MAX_TIME}초)")
data1 = load_data(file_path1, MAX_TIME)
data2 = load_data(file_path2, MAX_TIME)

if data1 is None or data2 is None:
    print("데이터 로드 실패로 종료합니다.")
    exit()

t1, x1, y1, z1, xd1, yd1, zd1 = data1
t2, x2, y2, z2, xd2, yd2, zd2 = data2

# ==========================================
# 4. 그래프 그리기
# ==========================================
# Figure 사이즈를 (7, 8)로 줄였습니다. (기존 10, 10)
plt.figure(figsize=(10, 7))

# --- x(t) plot ---
plt.subplot(3, 1, 1)
plt.plot(t1, x1, color='blue', linestyle='-', label='PD', linewidth=2.5)       # 굵게
plt.plot(t2, x2, color='green', linestyle='-', label='QPC', linewidth=2.5, alpha=0.8) # 굵게
plt.plot(t1, xd1, color='red', linestyle='--', label='Desired', linewidth=2.0) # 점선은 조금 얇게

plt.ylabel("x [m]")
plt.title(f"Position (0~{MAX_TIME}s)")
plt.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.6)
plt.xlim(0, MAX_TIME)

# --- y(t) plot ---
plt.subplot(3, 1, 2)
plt.plot(t1, y1, color='blue', linestyle='-', label='PD', linewidth=2.5)
plt.plot(t2, y2, color='green', linestyle='-', label='QPC', linewidth=2.5, alpha=0.8)
plt.plot(t1, yd1, color='red', linestyle='--', label='Desired', linewidth=2.0)

plt.ylabel("y [m]")
plt.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.6)
plt.xlim(0, MAX_TIME)

# --- z(t) plot ---
plt.subplot(3, 1, 3)
plt.plot(t1, z1, color='blue', linestyle='-', label='PD', linewidth=2.5)
plt.plot(t2, z2, color='green', linestyle='-', label='QPC', linewidth=2.5, alpha=0.8)
plt.plot(t1, zd1, color='red', linestyle='--', label='Desired', linewidth=2.0)

plt.xlabel("Time [s]")
plt.ylabel("z [m]")
plt.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.6)
plt.xlim(0, MAX_TIME)

plt.tight_layout()
plt.show()