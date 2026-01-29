import numpy as np
import os

# ==========================================
# 1. 설정 (User Configuration)
# ==========================================
# file_path_1 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp1.txt"
# file_path_2 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp2.txt"
# file_path_3 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp3.txt"

# file_path_1 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp1.txt"
# file_path_2 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp2.txt"
# file_path_3 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp3.txt"

# file_path_4 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp4.txt"
# file_path_5 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp5.txt"
# file_path_6 = "/home/cdsl/libfranka/a2_data/sim5/scurve/qp6.txt"

file_path_1 = "/home/cdsl/libfranka/a2_data/sim5/lspb/pd1.txt"
file_path_2 = "/home/cdsl/libfranka/a2_data/sim5/lspb/qp1.txt"
file_path_3 = "/home/cdsl/libfranka/a2_data/sim1/pd8.txt"
file_path_4 = "/home/cdsl/libfranka/a2_data/sim4/qp12.txt"

# file_path_1 = "/home/cdsl/libfranka/a2_data/sim5/scurve/pd1.txt"
# file_path_2 = "/home/cdsl/libfranka/a2_data/sim5/scurve/pd2.txt"
# file_path_3 = "/home/cdsl/libfranka/a2_data/sim5/scurve/pd3.txt"
# file_path_4 = "/home/cdsl/libfranka/a2_data/sim5/scurve/pd4.txt"
file_path_5 = "/home/cdsl/libfranka/a2_data/sim5/scurve/pd5.txt"
file_path_6 = "/home/cdsl/libfranka/a2_data/sim5/scurve/pd6.txt"

START_TIME = 7.0   # 계산 시작 시간
END_TIME   = 9.0  # 계산 종료 시간 (None = 끝까지)

# [중요] 텍스트 파일 내 데이터 컬럼 위치 설정 (0부터 시작)
# 예: time, x, y, z, x_d, y_d, z_d 순서라고 가정
COL_TIME = 0
COL_ACT  = [1, 2, 3] # Actual (x, y, z)
COL_DES  = [4, 5, 6] # Desired (x_d, y_d, z_d)

# ==========================================
# 2. RMSE 계산 함수
# ==========================================
def calculate_rmse(file_path, start_t, end_t):
    """
    파일을 읽어 (Actual - Desired) 오차를 직접 구한 뒤 RMSE를 계산
    """
    if not os.path.exists(file_path):
        print(f"[오류] 파일을 찾을 수 없습니다: {file_path}")
        return None

    # 오차를 저장할 리스트
    errors_x, errors_y, errors_z = [], [], []

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            # 헤더가 있다면 건너뛰기 (첫 줄이 숫자가 아니면 skip)
            # f.readline() 

            for line in f:
                parts = line.strip().split()
                # 데이터가 충분치 않은 빈 줄이나 깨진 줄 스킵
                if len(parts) <= max(COL_DES): 
                    continue
                
                try:
                    t = float(parts[COL_TIME])
                except ValueError:
                    continue # 헤더(문자열)인 경우 스킵

                # 시간 구간 필터링
                if t < start_t: continue
                if end_t is not None and t > end_t: break
                
                # ---------------------------------------------------------
                # [요청하신 부분] x, x_d를 직접 읽어서 오차 계산
                # ---------------------------------------------------------
                
                # 1. 실제값 (Actual) 읽기
                x_act = float(parts[COL_ACT[0]])
                y_act = float(parts[COL_ACT[1]])
                z_act = float(parts[COL_ACT[2]])

                # 2. 목표값 (Desired) 읽기
                x_des = float(parts[COL_DES[0]])
                y_des = float(parts[COL_DES[1]])
                z_des = float(parts[COL_DES[2]])

                # 3. 오차 계산 (Error = Actual - Desired)
                #    (x - x_d 형태로 직접 계산)
                errors_x.append(x_act - x_des)
                errors_y.append(y_act - y_des)
                errors_z.append(z_act - z_des)

        # 데이터가 없는 경우 처리
        if not errors_x:
            print(f"[경고] 해당 시간 구간({start_t}~{end_t}s)의 유효 데이터가 없습니다.")
            return None

        # Numpy 배열로 변환 (계산 속도 및 편의성)
        e_x = np.array(errors_x)
        e_y = np.array(errors_y)
        e_z = np.array(errors_z)

        # ---------------------------------------------------------
        # RMSE 계산: sqrt( mean( error^2 ) )
        # ---------------------------------------------------------
        rmse_x = np.sqrt(np.mean(e_x**2))
        rmse_y = np.sqrt(np.mean(e_y**2))
        rmse_z = np.sqrt(np.mean(e_z**2))
        
        # 3차원 전체 오차 (Euclidean Distance Error의 RMS)
        # norm_error^2 = ex^2 + ey^2 + ez^2
        total_sq_error = e_x**2 + e_y**2 + e_z**2
        rmse_total = np.sqrt(np.mean(total_sq_error))

        return rmse_x, rmse_y, rmse_z, rmse_total, len(e_x)

    except Exception as e:
        print(f"[오류] 데이터 처리 중 예외 발생: {e}")
        return None

# ==========================================
# 3. 실행 및 결과 출력
# ==========================================
print(f"--- RMSE Analysis (Time Window: {START_TIME} ~ {END_TIME} sec) ---")

res_1  = calculate_rmse(file_path_1, START_TIME, END_TIME)
res_2 = calculate_rmse(file_path_2, START_TIME, END_TIME)
res_3 = calculate_rmse(file_path_3, START_TIME, END_TIME)

res_4  = calculate_rmse(file_path_4, START_TIME, END_TIME)
res_5 = calculate_rmse(file_path_5, START_TIME, END_TIME)
res_6 = calculate_rmse(file_path_6, START_TIME, END_TIME)

def print_result(name, res):
    if res:
        rx, ry, rz, rt, count = res
        print(f"\n[{name}] Results (Samples: {count})")
        print(f"  * RMSE X : {rx:.6f} m")
        print(f"  * RMSE Y : {ry:.6f} m")
        print(f"  * RMSE Z : {rz:.6f} m")
        print(f"  * Total  : {rt:.6f} m (Euclidean)")
        return rt
    else:
        print(f"\n[{name}] 결과 없음")
        return None

val_1  = print_result("1 Control", res_1)
val_2 = print_result("2 Control", res_2)
val_3 = print_result("3 Control", res_3)
val_4  = print_result("4 Control", res_4)
val_5 = print_result("5 Control", res_5)
val_6 = print_result("6 Control", res_6)

# ==========================================
# 4. 성능 비교
# ==========================================
# if val_1 and val_2:
#     diff = val_pd - val_qpc
#     ratio = (diff / val_pd) * 100
#     print("\n" + "="*40)
#     if diff > 0:
#         print(f"Result: QPC is BETTER (Error reduced by {abs(ratio):.2f}%)")
#     else:
#         print(f"Result: PD is BETTER (Error reduced by {abs(ratio):.2f}%)")
#     print("="*40)