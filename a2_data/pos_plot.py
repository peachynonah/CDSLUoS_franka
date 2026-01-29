import matplotlib.pyplot as plt

# 파일 경로 설정 (데이터가 저장된 파일명으로 변경하세요)
# file_path = "/home/cdsl/libfranka/a2_data/sim6/quintic/qp11.txt"
file_path = "/home/cdsl/libfranka/3dof_sim1/qp8.txt"

# 데이터를 저장할 리스트 초기화
t_data = []
x_data, y_data, z_data = [], [], []
xd_data, yd_data, zd_data = [], [], [] # 목표값(desired) 비교용

try:
    with open(file_path, "r", encoding="utf-8") as f:
        # 첫 번째 줄(헤더: time x y z ...)은 건너뜁니다.
        header = f.readline()
        
        for line in f:
            parts = line.strip().split()
            
            # 빈 줄이거나 데이터가 부족한 경우 스킵 (헤더 제외 10개 열이 있어야 함)
            if len(parts) < 7:
                continue
            
            # 데이터 파싱 (문자열 -> 실수 변환)
            # data format: time(0) x(1) y(2) z(3) x_d(4) y_d(5) z_d(6) ...
            t_data.append(float(parts[0]))
            
            x_data.append(float(parts[1]))
            y_data.append(float(parts[2]))
            z_data.append(float(parts[3]))
            
            # 목표값(Desired)도 함께 그리기 위해 저장 (빨간 점선용)
            xd_data.append(float(parts[4]))
            yd_data.append(float(parts[5]))
            zd_data.append(float(parts[6]))

    # 그래프 그리기
    plt.figure(figsize=(10, 8))

    # x(t) plot
    plt.subplot(3, 1, 1)
    plt.plot(t_data, x_data, 'b-', label='Actual', linewidth=1.5)
    plt.plot(t_data, xd_data, 'r--', label='Desired', linewidth=1.0) # 목표값 비교
    plt.ylabel("x [m]")
    plt.title("Cartesian Position vs Time")
    plt.legend(loc='upper right')
    plt.grid(True)

    # y(t) plot
    plt.subplot(3, 1, 2)
    plt.plot(t_data, y_data, 'b-', label='Actual', linewidth=1.5)
    plt.plot(t_data, yd_data, 'r--', label='Desired', linewidth=1.0)
    plt.ylabel("y [m]")
    plt.legend(loc='upper right')
    plt.grid(True)

    # z(t) plot
    plt.subplot(3, 1, 3)
    plt.plot(t_data, z_data, 'b-', label='Actual', linewidth=1.5)
    plt.plot(t_data, zd_data, 'r--', label='Desired', linewidth=1.0)
    plt.xlabel("Time [s]")
    plt.ylabel("z [m]")
    plt.legend(loc='upper right')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

except FileNotFoundError:
    print(f"오류: '{file_path}' 파일을 찾을 수 없습니다. 파일 경로를 확인해주세요.")
except Exception as e:
    print(f"오류 발생: {e}")