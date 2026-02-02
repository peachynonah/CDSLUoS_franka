// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>  // [추가] 시간 측정을 위한 헤더
#include <string>  // [추가] 파일명 처리를 위해

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

  const double translational_stiffness{900.0};
  const double rotational_stiffness{20.0};
  Eigen::MatrixXd Kp(6,6), Kd(6,6);
  Kp.setZero();
  Kp.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  Kp.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  Kd.setZero();
  Kd.topLeftCorner(3, 3) << 1.5 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  Kd.bottomRightCorner(3, 3) << 1.5 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

  // ==============================================================================
  // Open file (기존 데이터 파일)
  // ==============================================================================
  std::ofstream myfile;
  myfile.open(argv[2], std::ios::out); 
  if (!myfile.is_open()) {
      std::cerr << "Failed to open file: " << argv[2] << std::endl;
      return -1;
  }
  myfile << "time x y z x_d y_d z_d \n";

  // [추가] 시간 측정용 파일 열기
  // 입력받은 파일명 뒤에 "_timing.txt"를 붙여서 생성
  std::string origin_name(argv[2]);
  std::string timing_name = origin_name + "_timing.txt";
  std::ofstream timing_file;
  timing_file.open(timing_name, std::ios::out);
  if (!timing_file.is_open()) {
      std::cerr << "Failed to open timing file." << std::endl;
      return -1;
  }
  // 헤더: 모델업데이트, 동역학계산, 필터링, 궤적생성, 제어(QP), 전체시간 (단위: us)
  timing_file << "t_model t_dynamics t_filter t_traj t_qp_control t_total\n";

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

    // 변수 초기화 (기존과 동일)
    Eigen::Vector3d position_target;
    position_target << 0.3, 0.005, 0.65;
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

    std::array<double, 6> x_a, x_b;       
    std::array<double, 6> y_a, y_b;       
    std::array<double, 6> y_b_dot;        

    double time_constant = 0.3;
    double T_traj = 5.0;

    ys_trajectory::QuinticPolynomialTrajectory traj_gen_quintic;
    ys_trajectory_lspb::LspbTrajectory3D traj_gen_lspb;
    ys_trajectory_scurve::ScurveAccTrapTrajectory3D traj_gen_scurve;

    const double jerk_max = 2.0;   
    const double acc_max = 1.0;   
    const double vel_max = 0.5;   
    const int mode = 1; 

    traj_gen_quintic.initTrajectory(init_position, position_target, T_traj); 
    traj_gen_lspb.init(init_position, position_target, acc_max, vel_max, mode, T_traj);
    traj_gen_scurve.init(init_position, position_target, jerk_max, acc_max, vel_max, mode, T_traj);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    double time = 0.0;
    static const double dt_threshold = 0.002;

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
      
      // [측정 시작] 전체 루프 시작 시간
      auto start_total = std::chrono::steady_clock::now();

      time += duration.toSec(); 
      
      if (duration.toSec() > dt_threshold) {
        std::cout << "[RT VIOLATION] dt: " << duration.toSec() << std::endl;
      }

      // [측정 포인트 1] 모델 업데이트 시작
      auto t_p1 = std::chrono::steady_clock::now();

      // -----------------------------------------------------------------------
      // [1. 상태 업데이트] Robot State & Dynamics
      // -----------------------------------------------------------------------
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 49> mass_array = model.mass(robot_state);
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      // [측정 포인트 2] 동역학/행렬 연산 시작
      auto t_p2 = std::chrono::steady_clock::now();

      Eigen::Matrix<double, 6, 6> lambda_inv = jacobian * M_bar_inv * jacobian.transpose();
      Eigen::Matrix<double, 6, 6> lambda = lambda_inv.inverse();
      Eigen::Matrix<double, 7, 6> J_inv = M_bar_inv * jacobian.transpose() * lambda;
      Eigen::Matrix<double, 6, 6> M_task = J_inv.transpose() * M_bar * J_inv;
      Eigen::Matrix<double, 6, 6> M_task_inv = M_task.inverse();

      Eigen::Matrix<double, 7, 1> V_ = generalizedCrossProduct(jacobian);
      double M_null = V_.transpose() * M_bar * V_;
      double M_null_inv = 1/M_null;
      Eigen::Matrix<double, 1, 7> J_N = M_null_inv * (V_.transpose() * M_bar);

      Eigen::Matrix<double, 7, 7> NullProj = Eigen::Matrix<double, 7, 7>::Identity() - (J_inv * jacobian);
      Eigen::Matrix<double, 7, 1> K0;
      K0.setConstant(0.1);
      Eigen::Matrix<double, 7, 1> v0 = K0.asDiagonal() * dq;
      double f_N = J_N * v0;

      // [측정 포인트 3] 필터링 시작
      auto t_p3 = std::chrono::steady_clock::now();

      Eigen::Matrix<double, 6, 1> position_Xdot = jacobian * dq;
      for (int i = 0; i < 6; i++) {
          x_b[i] = position_Xdot(i); 
          x_a[i] = uhat_update(i); 
          y_a[i] = calculate_lowpass_filter(x_a[i], y_a_prev[i], time_constant);
          y_b[i] = calculate_lowpass_filter(x_b[i], y_b_prev[i], time_constant);
          y_b_dot[i] = (x_b[i] - y_b[i]) / time_constant;
          uhat_force(i) = y_a[i];       
          position_Xddot(i) = y_b_dot[i]; 
      }
    
      // [측정 포인트 4] 궤적 생성 시작
      auto t_p4 = std::chrono::steady_clock::now();

      // -----------------------------------------------------------------------
      // [2. Trajectory Generator]
      // -----------------------------------------------------------------------
      if (time >= 3.0) {
          double t_run = time - 3.0;
          ys_trajectory_scurve::TrajectoryState3D_Jerk target = traj_gen_scurve.compute(t_run);          
          position_d = target.position;
          velocity_d = target.velocity;
      } else {
          position_d = init_position;
          velocity_d.setZero();
      }

      // [측정 포인트 5] 제어(QP) 시작
      auto t_p5 = std::chrono::steady_clock::now();

      // -----------------------------------------------------------------------
      // [3. Error Calculation] & [4. Task Space Controller]
      // -----------------------------------------------------------------------
      Eigen::Matrix<double, 6, 1> error;
      Eigen::Matrix<double, 6, 1> error_dot;
      
      error.head(3) = position - position_d;
      // Eigen::Matrix<double, 6, 1> current_velocity = jacobian * dq;
      Eigen::Matrix<double, 6, 1> current_velocity = position_Xdot;
      error_dot.head(3) = current_velocity.head(3) - velocity_d;

      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      error.tail(3) << -transform.linear() * error.tail(3); 
      error_dot.tail(3) = current_velocity.tail(3); 

      Eigen::VectorXd tau_task(7), tau_d(7);
      Eigen::VectorXd OuterloopForce = -Kp * error - Kd * error_dot;

      Eigen::Matrix<double, 7, 7> P = 2.0 * (J_inv * J_inv.transpose() + V_ * V_.transpose());
      Eigen::Matrix<double, 7, 1> q_qp = -2.0 * (J_inv * M_task * (position_Xddot - uhat_force)) - 2.0 * (f_N * V_);
      // Eigen::Matrix<double, 7, 1> bound = Eigen::Matrix<double, 7, 1>::Constant(2.0);
      Eigen::Matrix<double, 7, 1> bound;
      // bound << 8.7, 8.7, 8.7, 8.7, 8, 8, 8;
      bound << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0;
      Eigen::VectorXd OuterloopTorque = jacobian.transpose() * lambda * OuterloopForce;

      Eigen::Matrix<double, 7, 1> lb  = OuterloopTorque - bound;
      Eigen::Matrix<double, 7, 1> ub  = OuterloopTorque + bound;

      // 여기가 가장 무거울 것으로 예상됨 (QP Solver)
      Eigen::Matrix<double, 7, 1> qpc_output_tau = solveqpOASES(P, q_qp, lb, ub);
      Eigen::Matrix<double, 7, 1> control_input = OuterloopTorque - qpc_output_tau;
// QPC
      tau_d << control_input;
      uhat_update = M_task_inv* J_inv.transpose() * control_input;
// PD control
      // Eigen::Matrix<double, 7, 1> Nullctl = NullProj * v0; // Null space control
      // tau_d << OuterloopTorque + Nullctl;
      
      // [측정 종료] 전체 루프 종료
      auto end_total = std::chrono::steady_clock::now();

      // [시간 계산] (Microseconds 단위)
      long long d_model = std::chrono::duration_cast<std::chrono::microseconds>(t_p2 - t_p1).count();
      long long d_dyn   = std::chrono::duration_cast<std::chrono::microseconds>(t_p3 - t_p2).count();
      long long d_filt  = std::chrono::duration_cast<std::chrono::microseconds>(t_p4 - t_p3).count();
      long long d_traj  = std::chrono::duration_cast<std::chrono::microseconds>(t_p5 - t_p4).count();
      long long d_qp    = std::chrono::duration_cast<std::chrono::microseconds>(end_total - t_p5).count();
      long long d_total = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start_total).count();

      // [파일 저장]
      // std::endl 대신 \n을 사용하여 불필요한 버퍼 플러시 방지 (속도 저하 방지)
      timing_file << d_model << " " 
                  << d_dyn << " " 
                  << d_filt << " " 
                  << d_traj << " " 
                  << d_qp << " " 
                  << d_total << "\n";

      // -----------------------------------------------------------------------
      // [5. Data Logging] (기존 데이터)
      // -----------------------------------------------------------------------
      myfile << time << " "
            << position[0] << " " << position[1] << " " << position[2] << " "
            << position_d[0] << " " << position_d[1] << " " << position_d[2] << " "
            << current_velocity[0] << " " << current_velocity[1] << " " << current_velocity[2] << " "
            << orientation.x() << " " << orientation.y() << " " << orientation.z() << " "
            << orientation_d.x() << " " << orientation_d.y() << " " << orientation_d.z() << " "
            // << qpc_output_tau[0] << " " << qpc_output_tau[1] << " " << qpc_output_tau[2] << " "
            // << qpc_output_tau[3] << " " << qpc_output_tau[4] << " " << qpc_output_tau[5] << " " << qpc_output_tau[6]
            << control_input[0] << " " << control_input[1] << " " << control_input[2] << " "
            << control_input[3] << " " << control_input[4] << " " << control_input[5] << " " << control_input[6]            
            << "\n";

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);

      return tau_d_rate_limited;
    };

    std::cout << "WARNING: Robot will move! Radius=10cm Circle." << std::endl
              << "Press Enter to start..." << std::endl;
    std::cin.ignore();
    
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
    myfile.close(); 
    timing_file.close(); // [추가] 닫기
    return -1;
  }

  myfile.close(); 
  timing_file.close(); // [추가] 닫기
  return 0;
}

double calculate_lowpass_filter(double input, double& prev, double time_constant) {
    // Franka Sampling Time = 0.001 (1ms)
    const double Ts = 0.001;
    // double alpha = Ts / (time_constant + Ts);
    // double alpha = 0.00332225913;
    double alpha = Ts / (time_constant + Ts);
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