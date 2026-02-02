// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

#include "examples_common.h"
#include <qpOASES.hpp>
#include "ys_trajectory_generator.h"
#include "lspb.h"
#include "scurve.h"

double calculate_lowpass_filter(double input, double& prev, double time_constant);
Eigen::VectorXd generalizedCrossProduct(const Eigen::Matrix<double, 6, 7>& matrix);
Eigen::Matrix<double, 7, 1> solveqpOASES(const Eigen::Matrix<double, 7, 7> &P, const Eigen::Matrix<double, 7, 1> &q,
      const Eigen::Matrix<double, 7, 1> &lb, const Eigen::Matrix<double, 7, 1> &ub);

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <output-filename>" << std::endl;
    return -1;
  }

  // ==============================================================================
  // Control gain setting (Stiffness & Damping)
  // ==============================================================================
  //변경
  const double translational_stiffness{900.0};
  const double rotational_stiffness{20.0};
  Eigen::MatrixXd Kp(6,6), Kd(6,6);
  Kp.setZero();
  Kp.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  Kp.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  // Critical Damping: 2 * sqrt(k)
  Kd.setZero();
  Kd.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  Kd.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

  // ==============================================================================
  // Open file
  // ==============================================================================
  std::ofstream myfile; 
  myfile.open(argv[2], std::ios::out); 
  if (!myfile.is_open()) {
      std::cerr << "Failed to open file: " << argv[2] << std::endl;
      return -1;
  }
  // 헤더 작성 (데이터 분석 시 편리)
  myfile << "time x y z x_d y_d z_d \n";
  // roll pitch yaw roll_d pitch_d yaw_d\n";

  try {
    // 1. 로봇 연결
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);

    // 2. 모델 로드
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // 3. 초기 위치 설정
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d init_position(initial_transform.translation());
    Eigen::Quaterniond init_orientation(initial_transform.linear());

    //변수 초기화
    Eigen::Vector3d position_target;
    // position_target << 0.3, 0.005, 0.5; //, -0.7, 0.002, 3.0; 맨처음 설정 (PDQP2)
    position_target << 0.3, 0.005, 0.65; //, -0.7, 0.002, 3.0; (3번)
    // position_target << 0.32, 0.0, 0.65; //4번
    // position_target <<  0.32, 0.004, 0.65;

    // position_target << 0.01, -0.048, 0.7;
    Eigen::Vector3d velocity_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_d = init_position;
    Eigen::Vector3d velocity_d = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_d = init_orientation;

    Eigen::Matrix<double, 6, 1> uhat_force;
    Eigen::Matrix<double, 6, 1> uhat_update;
    Eigen::Matrix<double, 7, 1> disturbance_;    
    disturbance_.setZero();
    uhat_force << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 6, 1> position_Xdot;
    Eigen::Matrix<double, 6, 1> position_Xddot;

    static std::array<double, 6> y_a_prev = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static std::array<double, 6> y_b_prev = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // [3] 연산용 임시 변수 (std::array)
    std::array<double, 6> x_a, x_b;       // 필터 입력 (Raw)
    std::array<double, 6> y_a, y_b;       // 필터 출력 (Filtered)
    std::array<double, 6> y_b_dot;        // 미분 추정치

      //변경
    double time_constant = 0.3; // 0.1, 0.2, 0.005 ms
    double T_traj = 5.0; //5초 동안 이동

    ys_trajectory::QuinticPolynomialTrajectory traj_gen_quintic;
    ys_trajectory_lspb::LspbTrajectory3D traj_gen_lspb;
    ys_trajectory_scurve::ScurveAccTrapTrajectory3D traj_gen_scurve;

  // LSPB, Scurve(init)
    const double jerk_max = 2.0;   // [m/s^3] 2.0, 1.0, 0.5, 5.0, 10.0, 20.0
    const double acc_max = 1.0;   // [m/s^2]
    const double vel_max = 0.5;   // [m/s]
    const int mode = 1; // mode=1 (시간 고정)

  // 궤적 생성기 초기화 (프로그램 시작 시 1번만)
    traj_gen_quintic.initTrajectory(init_position, position_target, T_traj); //5차 다항식  
    traj_gen_lspb.init(init_position, position_target, acc_max, vel_max, mode, T_traj);
    traj_gen_scurve.init(init_position, position_target, jerk_max, acc_max, vel_max, mode, T_traj);

  // 충돌 감지 임계값 설정 (개발 중에는 높게 설정)
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    // 시간 변수
    double time = 0.0;
    static const double dt_threshold = 0.002;  // 2ms 이상 지연 시 경고

    //한번만 계산하는 변수
    std::array<double, 49> mass_array_init = model.mass(initial_state);
    Eigen::Map<const Eigen::Matrix<double, 7, 7>> M_bar_map(mass_array_init.data());
    const Eigen::Matrix<double, 7, 7> M_bar = M_bar_map;
    const Eigen::Matrix<double, 7, 7> M_bar_inv = M_bar.inverse();

    // ==============================================================================
    // [메인 제어 루프]
    // ==============================================================================
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      
      time += duration.toSec(); // 누적 시간 업데이트
      static double last_print_time = 0.0; 

      // RT Violation Check
      if (duration.toSec() > dt_threshold) {
        std::cout << "[RT VIOLATION] dt: " << duration.toSec() << std::endl;
      }
      // -----------------------------------------------------------------------
      // [1. 상태 업데이트] Robot State & Dynamics
      // -----------------------------------------------------------------------
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 49> mass_array = model.mass(robot_state);
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      
      // Eigen 변환
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      // ---------Operational Space Inertia Matrix (Lambda)--------
      //J_inv(수도인버스) = M_bar_inv @ J.T @ np.linalg.inv(J @ M_bar_inv @ J.T) 아래 수식에서 mass만 M_bar로 바꾸면 됨
      // Lambda = (J * M^-1 * J^T)^-1

      Eigen::Matrix<double, 6, 6> lambda_inv = jacobian * M_bar_inv * jacobian.transpose();
      Eigen::Matrix<double, 6, 6> lambda = lambda_inv.inverse();
      Eigen::Matrix<double, 7, 6> J_inv = M_bar_inv * jacobian.transpose() * lambda;
      Eigen::Matrix<double, 6, 6> M_task = J_inv.transpose() * M_bar * J_inv;
      Eigen::Matrix<double, 6, 6> M_task_inv = M_task.inverse();

      //-------V구하기 JV = 0이 되는 V계산 (7x6)----------
      Eigen::Matrix<double, 7, 1> V_ = generalizedCrossProduct(jacobian);
      double M_null = V_.transpose() * M_bar * V_;
      double M_null_inv = 1/M_null;
      Eigen::Matrix<double, 1, 7> J_N = M_null_inv * (V_.transpose() * M_bar);

      //---------null space control 관련 변수 계산
      Eigen::Matrix<double, 7, 7> NullProj = Eigen::Matrix<double, 7, 7>::Identity() - (J_inv * jacobian);
      Eigen::Matrix<double, 7, 1> K0;
      //변경
      // K0.setConstant(10.0);
      K0.setConstant(0.1);
      Eigen::Matrix<double, 7, 1> v0 = K0.asDiagonal() * dq;
      double f_N = J_N * v0;

      // -------[input, end_dot에 LPF 적용]-------
      //이전에 초기화된 input 필요 + end_dot(X_dot)은 없어서 J*dq 로 구해야 함
      // 1. 현재 카르테시안 속도 계산 (6x1 벡터)
      Eigen::Matrix<double, 6, 1> position_Xdot = jacobian * dq;

      // 2. 필터링 및 미분치 추정 루프
      for (int i = 0; i < 6; i++) {
          // [데이터 대입]
          x_b[i] = position_Xdot(i);  // Eigen(i) -> Array[i]
          x_a[i] = uhat_update(i);    // (이전에 계산된 F_cmd가 들어있어야 함)

          // [함수 호출]
          // y_a_prev[i]는 참조(&)로 전달되어 함수 내부에서 값이 갱신됨
          y_a[i] = calculate_lowpass_filter(x_a[i], y_a_prev[i], time_constant);
          y_b[i] = calculate_lowpass_filter(x_b[i], y_b_prev[i], time_constant);
          
          // [미분치(가속도) 추정] (Raw - Filtered) / Tc 방식
          // 혹은 일반적인 차분: (y_b[i] - y_b_prev[i]) / Ts 등을 쓸 수도 있음
          // 여기서는 작성하신 코드를 따름:
          y_b_dot[i] = (x_b[i] - y_b[i]) / time_constant;

          // [결과 저장]
          uhat_force(i) = y_a[i];       // 필터링된 힘을 다시 Eigen 벡터에 저장
          position_Xddot(i) = y_b_dot[i]; // 추정된 가속도 저장
      }
    
      // -----------------------------------------------------------------------
      // [2-a. Trajectory Generator] 궤적 생성 (원형 운동)
      // -----------------------------------------------------------------------
      // constexpr double radius = 0.1;       // 반지름 10cm
      // constexpr double period = 5.0;       // 주기 5초
      // const double omega = 2.0 * M_PI / period;
      
      // // 궤적 시작 전 3초 대기 (Safety)
      // if (time >= 3.0) {
      //     double t_run = time - 3.0;
      //     // X-Z 평면 원형 궤적
      //     // x(t) = init_x + r * sin(w*t)
      //     // z(t) = init_z + r * (cos(w*t) - 1) -> 아래쪽으로 원 그림
          
      //     double angle = omega * t_run;

      //     // 위치 목표
      //     position_d[0] = init_position[0] + radius * std::sin(angle);
      //     position_d[1] = init_position[1];
      //     position_d[2] = init_position[2] + radius * (std::cos(angle) - 1.0);
          
      //     // 속도 목표 (미분)
      //     velocity_d[0] = radius * omega * std::cos(angle);
      //     velocity_d[1] = 0.0;
      //     velocity_d[2] = -radius * omega * std::sin(angle);
      // } else {
      //     // 대기 상태 (초기 위치 유지)
      //     position_d = init_position;
      //     velocity_d.setZero();
      // }

      // // -----------------------------------------------------------------------
      // // [2-b. Trajectory Generator] 궤적 생성 (5차 다항식)
      // // -----------------------------------------------------------------------
      // 궤적 시작 전 3초 대기 (Safety)
      if (time >= 3.0) {
          double t_run = time - 3.0;
          ys_trajectory::TrajectoryState target = traj_gen_quintic.computeState(t_run); //5차 다항식
          // ys_trajectory_lspb::TrajectoryState3D target = traj_gen_lspb.compute(t_run); //LSPB
          // ys_trajectory_scurve::TrajectoryState3D_Jerk target = traj_gen_scurve.compute(t_run);          

          position_d = target.position;
          velocity_d = target.velocity;

      } else {
          // 대기 상태 (초기 위치 유지)
          position_d = init_position;
          velocity_d.setZero();
      }

      // -----------------------------------------------------------------------
      // [3. Error Calculation] 에러 계산
      // -----------------------------------------------------------------------
      Eigen::Matrix<double, 6, 1> error;
      Eigen::Matrix<double, 6, 1> error_dot;
      
      // Position Error (x - x_d)
      error.head(3) = position - position_d;
      
      // Velocity Error (v - v_d)
      // 현재 Cartesian Velocity = J * dq
      Eigen::Matrix<double, 6, 1> current_velocity = jacobian * dq;
      error_dot.head(3) = current_velocity.head(3) - velocity_d;

      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      error.tail(3) << -transform.linear() * error.tail(3); // Base Frame으로 변환
      error_dot.tail(3) = current_velocity.tail(3); // 회전 속도 목표는 0이라고 가정

      // -----------------------------------------------------------------------
      // [4. Task Space Controller] 제어 입력 계산
      // -----------------------------------------------------------------------
      Eigen::VectorXd tau_task(7), tau_d(7);
      Eigen::VectorXd OuterloopForce = -Kp * error - Kd * error_dot;

      Eigen::Matrix<double, 7, 7> P = 2.0 * (J_inv * J_inv.transpose() + V_ * V_.transpose());
      Eigen::Matrix<double, 7, 1> q_qp = -2.0 * (J_inv * M_task * (position_Xddot - uhat_force)) - 2.0 * (f_N * V_);
  //변경
      Eigen::Matrix<double, 7, 1> bound = Eigen::Matrix<double, 7, 1>::Constant(2.0);
      Eigen::VectorXd OuterloopTorque = jacobian.transpose() * lambda * OuterloopForce;

      Eigen::Matrix<double, 7, 1> lb  = OuterloopTorque - bound; //lb = OuterloopTorque - ub => J^T*M_task*vt - 100
      Eigen::Matrix<double, 7, 1> ub  = OuterloopTorque + bound; //ub = OuterloopTorque - lb => J^T*M_task*vt + 100

      Eigen::Matrix<double, 7, 1> qpc_output_tau = solveqpOASES(P, q_qp, lb, ub);
      Eigen::Matrix<double, 7, 1> control_input = OuterloopTorque - qpc_output_tau;
//qpc로 적용 
      tau_d << control_input;
      uhat_update = M_task_inv* J_inv.transpose() * control_input; //다음 lpf에 넣을 input

      // disturbance_[3] = std::sin(M_PI * /*f*/2 * time);
      // control_input += disturbance_;

      // Torque = J^T * v_t (Nullspace 제외 간단 버전)
      // 정확한 FL: tau = J^T * Lambda * OuterloopInput + Coriolis
      // 여기서는 관성 행렬(Lambda)을 곱해주는 것이 일반적입니다.

// Null space control
      // Eigen::Matrix<double, 7, 1> Nullctl = NullProj * v0;

// PD control
      // tau_d << OuterloopTorque + Nullctl;

//FL control 
      // tau_task << OuterloopTorque; // tau_task += disturbance_;
      // tau_d << tau_task + coriolis; 

      // -----------------------------------------------------------------------
      // [5. Data Logging] 데이터 출력 및 저장
      // -----------------------------------------------------------------------
      
      // 데이터 출력: (현재시간 - 마지막출력시간)이 2.0초보다 크면 실행
      // if (time - last_print_time >= 0.5) {
      //     std::cout << "Current Time: " << time << "\n"
      //               // << " | Pos X: " << position[0] 
      //               // << " | Pos Y: " << position[1]
      //               // << " | Pos Z: " << position[2] << std::endl;
      //               << "qpc_output_tau : " << qpc_output_tau << "\n"
      //               << "tau_d :" << tau_d << "\n"
      //               << "OuterloopTorque : " << OuterloopTorque << "\n"
      //               << "lb : " << lb << "\n"
      //               << "ub : " << ub << "\n"
      //               << std::endl;
      //     // 마지막 출력 시간을 현재 시간으로 갱신
      //     last_print_time = time;
      // }
      
      // 데이터 저장: 시간, 현재위치(x,y,z), 목표위치(x,y,z), 에러(x,y,z) 저장
      myfile << time << " "
             << position[0] << " " << position[1] << " " << position[2] << " "
             << position_d[0] << " " << position_d[1] << " " << position_d[2] << " "
            //  << orientation.x() << " " << orientation.y() << " " << orientation.z() << " " << orientation.w() << " "
            //  << orientation_d.x() << " " << orientation_d.y() << " " << orientation_d.z() << " " << orientation_d.w() 
             << "\n";
             //  << error[0] << " " << error[1] << " " << error[2] << "\n";
      // -----------------------------------------------------------------------
      // [6. Torque Return]
      // -----------------------------------------------------------------------
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      // return tau_d_array;

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);

      return tau_d_rate_limited;
    };

    // 로봇 제어 시작
    std::cout << "WARNING: Robot will move! Radius=10cm Circle." << std::endl
              << "Press Enter to start..." << std::endl;
    std::cin.ignore();
    
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    myfile.close(); // 예외 발생 시 파일 닫기
    return -1;
  }

  myfile.close(); // 정상 종료 시 파일 닫기
  return 0;
}

double calculate_lowpass_filter(double input, double& prev, double time_constant) {
    // Franka Sampling Time = 0.001 (1ms)
    // const double Ts = 0.001;
    // double alpha = Ts / (time_constant + Ts);
    double alpha = 0.00332225913;
    // 필터링 수식: y_k = alpha * u_k + (1 - alpha) * y_k-1
    double output = alpha * input + (1.0 - alpha) * prev;
    // [중요] 다음 루프를 위해 이전 값을 현재 값으로 업데이트 (참조 변수라 원본이 바뀜)
    prev = output;
    
    return output;
}

Eigen::VectorXd generalizedCrossProduct(const Eigen::Matrix<double, 6, 7>& matrix) {
  Eigen::VectorXd result(7);
  for (int i = 0; i < 7; ++i) {
    Eigen::Matrix<double, 6, 6> subMatrix;
    int col = 0;
    for (int j = 0; j < 7; ++j) {
      if (i == j) continue;
      subMatrix.col(col++) = matrix.col(j);
    }
    result(i) = std::pow(-1, i) * subMatrix.determinant();
  }
  return result;
}

Eigen::Matrix<double, 7, 1> solveqpOASES(
      const Eigen::Matrix<double, 7, 7> &P, const Eigen::Matrix<double, 7, 1> &q,
      const Eigen::Matrix<double, 7, 1> &lb, const Eigen::Matrix<double, 7, 1> &ub) {
    
    qpOASES::real_t H[49];
    qpOASES::real_t g[7];
    qpOASES::real_t lb_in[7];
    qpOASES::real_t ub_in[7];

    for (int r = 0; r < 7; ++r) {
        for (int c = 0; c < 7; ++c) {
            H[r*7 + c] = static_cast<qpOASES::real_t>(P(r,c));
        }
        g[r]  = static_cast<qpOASES::real_t>(q(r));
        lb_in[r] = static_cast<qpOASES::real_t>(lb(r));
        ub_in[r] = static_cast<qpOASES::real_t>(ub(r));
    }
    
    // 일반 QProblem 클래스 (변수 7개, 제약조건 0개)
    static qpOASES::QProblem qp_solver(7, 0);
    
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    options.enableFlippingBounds = qpOASES::BT_TRUE;
    qp_solver.setOptions(options);
    int nWSR = 150;
    
    // init(H, g, A, lb, ub, lbA, ubA, nWSR)
    qp_solver.init(H, g, nullptr, lb_in, ub_in, nullptr, nullptr, nWSR);
    qpOASES::real_t sol[7] = {0};
    qp_solver.getPrimalSolution(sol);
    
    Eigen::Matrix<double,7,1> x;
    for(int i=0; i<7; ++i) x(i) = sol[i];
    return x;
}