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
Eigen::VectorXd generalizedCrossProduct(const Eigen::Matrix<double, 1, 3>& matrix);
Eigen::Matrix<double, 3, 1> solveqpOASES(const Eigen::Matrix<double, 3, 3> &P, const Eigen::Matrix<double, 3, 1> &q,
      const Eigen::Matrix<double, 3, 1> &lb, const Eigen::Matrix<double, 3, 1> &ub);

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <output-filename>" << std::endl;
    return -1;
  }

  // const double translational_stiffness{900.0};
  // const double rotational_stiffness{20.0};
  // Eigen::MatrixXd Kp(6,6), Kd(6,6);
  // Kp.setZero();
  // Kp.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  // Kp.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  // Kd.setZero();
  // Kd.topLeftCorner(3, 3) << 1.5 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  // Kd.bottomRightCorner(3, 3) << 1.5 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);


//joint control PD gain
  Eigen::VectorXd kp_vec(7);
  Eigen::VectorXd kd_vec(7);
  kp_vec << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
  kd_vec <<  50.0,  50.0,  50.0,  50.0,  30.0,  25.0, 15.0;

  Eigen::MatrixXd pd_Kp(7,7), pd_Kd(7,7);
  pd_Kp = kp_vec.asDiagonal();
  pd_Kd = kd_vec.asDiagonal();

//task control PD gain
  double Kp{900.0};
  double Kd{1.5*900.0};

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

  // [추가] 시간 측정용 파일 열기 / 입력받은 파일명 뒤에 "_timing.txt"를 붙여서 생성
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
    double init_z = initial_transform.translation().z();
    Eigen::Quaterniond init_orientation(initial_transform.linear());

    std::array<double, 7> initial_joint_position = initial_state.q;
    Eigen::Matrix<double, 7, 1> initial_joint_velocity;
    initial_joint_velocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // 변수 초기화
    Eigen::Vector3d position_target;
    position_target << 0.3, 0.005, 0.65;
    Eigen::Vector3d velocity_target = Eigen::Vector3d::Zero(); //target값은 궤적 생성을 위해서만 사용됩니다. scurve등의 함수 변형없이 사용하려다보니 변수형은 그대로 유지함.

    double position_d = init_position(2); // 실제 상태오차 계산 시 적용되는 z축변수
    double velocity_d = 0;
    Eigen::Quaterniond orientation_d = init_orientation; //(m=1인 상황이므로 사실상 자세제어는 없음)

    Eigen::VectorXd dq_3(7);

    double uhat_force = 0;
    double uhat_update;
    Eigen::Matrix<double, 3, 1> disturbance_;
    disturbance_.setZero();
    double position_Xdot, position_Xddot;

    static double y_a_prev = 0.0;
    static double y_b_prev = 0.0;

    double x_a, x_b;       
    double y_a, y_b;       
    double y_b_dot;        

    double time_constant = 0.3;
    double T_traj = 5.0; // 5초동안 궤적
    double time_print_timer = 0.0;

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

  // 인덱스 정의
    std::vector<int> idx_lock   = {0, 2, 4, 6}; // 고정할 관절 (1, 3, 5, 7번 joint)
    std::vector<int> idx_active = {1, 3, 7};    // 제어할 관절 (2, 4, 6번 joint)

    std::array<double, 49> mass_array_init = model.mass(initial_state);
    Eigen::Map<const Eigen::Matrix<double, 7, 7>> M_bar_map(mass_array_init.data());
    // const Eigen::Matrix<double, 7, 7> M_bar = M_bar_map;
    // const Eigen::Matrix<double, 7, 7> M_bar_inv = M_bar.inverse();
    Eigen::Matrix<double, 3, 3> M_bar;
      for(int i=0; i<3; i++) {
        int r = idx_active[i];
          for(int j=0; j<3; j++) {
              int c = idx_active[j]; // Col 인덱스 (2, 4, 6번 joint)
              M_bar(i, j) = M_bar_map(r, c);
          }
      }
    const Eigen::Matrix<double, 3, 3> M_bar_inv = M_bar.inverse();

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

      // -----------------------------------------------------------------------
      // 3축에 대한 동역학으로 변환 
      // Mass: (7x7) => (3x3),       Coriolis: (7x1) => (3x1),       Jacobian: (6x7) => (1x3)
      // -----------------------------------------------------------------------      
      // --- (1) 동역학 데이터 추출 (Slicing) ---
      Eigen::Matrix3d mass_3;
      Eigen::Vector3d coriolis_3;

      for(int i=0; i<3; i++) {
          int r = idx_active[i]; // Row 인덱스 (2, 4, 6번 joint)
          coriolis_3(i)  = coriolis(r);
          dq_3(i)  = robot_state.dq[r];

          for(int j=0; j<3; j++) {
              int c = idx_active[j]; // Col 인덱스 (2, 4, 6번 joint)
              mass_3(i, j) = mass(r, c);
          }
      }
      // --- (2) 자코비안 추출 (Z축 only) ---
      // 전체 Jacobian 구조: [vx, vy, vz, wx, wy, wz]^T 우리는 vz(인덱스 2)만 필요함.
      Eigen::MatrixXd jacobian_3(1, 3);

      for(int i=0; i<3; i++) {
          int c = idx_active[i]; // Col 인덱스 (2, 4, 6번 joint)
          jacobian_3(0, i) = jacobian(2, c); // 2번째 행(Z축)만 가져옴
      }

      double lambda_inv = (jacobian_3 * M_bar_inv * jacobian_3.transpose()).value();
      double lambda = 1/lambda_inv;

      // Eigen::Matrix<double, 7, 6> J_inv = M_bar_inv * jacobian.transpose() * lambda;
      // Eigen::Matrix<double, 6, 6> M_task = J_inv.transpose() * M_bar * J_inv;
      // Eigen::Matrix<double, 6, 6> M_task_inv = M_task.inverse();

      Eigen::Matrix<double, 3, 1> J_inv = M_bar_inv * jacobian_3.transpose() * lambda;
      double M_task = (J_inv.transpose() * M_bar * J_inv).value();
      double M_task_inv = 1/M_task;
      Eigen::Matrix<double, 3, 1> V_ = generalizedCrossProduct(jacobian_3);
      double M_null = V_.transpose() * M_bar * V_;
      double M_null_inv = 1/M_null;
      Eigen::Matrix<double, 1, 3> J_N = M_null_inv * (V_.transpose() * M_bar);

      Eigen::Matrix<double, 3, 3> NullProj = Eigen::Matrix<double, 3, 3>::Identity() - (J_inv * jacobian_3);
      Eigen::Matrix<double, 3, 1> K0;
      K0.setConstant(0.1);
      Eigen::Matrix<double, 3, 1> v0 = K0.asDiagonal() * dq_3;
      double f_N = J_N * v0;

      // [측정 포인트 3] 필터링 시작
      auto t_p3 = std::chrono::steady_clock::now();
      Eigen::Matrix<double, 6, 1> current_velocity_display = jacobian * dq;
      double position_Xdot = (jacobian_3 * dq_3).value();
          x_b = position_Xdot;
          x_a = uhat_update;
          y_a = calculate_lowpass_filter(x_a, y_a_prev, time_constant);
          y_b = calculate_lowpass_filter(x_b, y_b_prev, time_constant);
          y_b_dot = (x_b - y_b) / time_constant;
          uhat_force = y_a;
          position_Xddot = y_b_dot;
    
      // [측정 포인트 4] 궤적 생성 시작
      auto t_p4 = std::chrono::steady_clock::now();
      // -----------------------------------------------------------------------
      // [2. Trajectory Generator]
      // -----------------------------------------------------------------------
      if (time >= 3.0) {
          double t_run = time - 3.0;  // 3초 뒤부터 궤적 시작
          ys_trajectory_scurve::TrajectoryState3D_Jerk target = traj_gen_scurve.compute(t_run);
          position_d = target.position(2); //z축 정보만 받아오기
          velocity_d = target.velocity(2);
      } else {
          position_d = init_z;
          velocity_d = 0;
      }

      // -----------------------------------------------------------------------
      // [3. Joint Space PD Controller_초기위치 유지]
      // -----------------------------------------------------------------------
      Eigen::Matrix<double, 7, 1> joint_error;
      Eigen::Matrix<double, 7, 1> joint_error_dot;

      // joint_error = q - initial_joint_position; 에서 타입 불일치.
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_init_map(initial_joint_position.data());
      joint_error = q - q_init_map;
      joint_error_dot = dq - initial_joint_velocity;

      Eigen::VectorXd tau_pd(7);
      Eigen::VectorXd tau_pd_total(7);
      tau_pd.setZero();
      tau_pd_total.setZero();

      tau_pd_total = - pd_Kp*joint_error - pd_Kd*joint_error_dot;
      tau_pd = tau_pd_total;
      for (int i : idx_lock) {
          tau_pd(i) = 0.0;// (2, 4, 6번째 토크만 적용..)
      }

      // [측정 포인트 5] 제어(QP) 시작
      auto t_p5 = std::chrono::steady_clock::now();
      // -----------------------------------------------------------------------
      // [4. QP-based Task Space Controller]
      // -----------------------------------------------------------------------
      double error, error_dot;
      double current_velocity = position_Xdot;

      error = position(2) - position_d;
      error_dot = current_velocity - velocity_d;

      Eigen::VectorXd tau_qp(7);
      tau_qp.setZero();
      double OuterloopForce = -Kp * error - Kd * error_dot;

      Eigen::Matrix<double, 3, 3> P = 2.0 * (J_inv * J_inv.transpose() + V_ * V_.transpose());
      Eigen::Matrix<double, 3, 1> q_qp = -2.0 * (J_inv * M_task * (position_Xddot - uhat_force)) - 2.0 * (f_N * V_);
      
      Eigen::Matrix<double, 3, 1> bound;
      // bound << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0;
      bound << 87.0, 87.0, 12.0;// 2,4,6번 joint만 적용(index로는 135)
      Eigen::VectorXd OuterloopTorque = jacobian_3.transpose() * lambda * OuterloopForce;
      Eigen::Matrix<double, 3, 1> lb  = OuterloopTorque - bound;
      Eigen::Matrix<double, 3, 1> ub  = OuterloopTorque + bound;

      Eigen::Matrix<double, 3, 1> qpc_output_tau = solveqpOASES(P, q_qp, lb, ub);
      Eigen::Matrix<double, 3, 1> control_input = OuterloopTorque - qpc_output_tau;
// QPC
      // tau_d << control_input;
      tau_qp(1) = control_input(0); // Joint 2
      tau_qp(3) = control_input(1); // Joint 4
      tau_qp(5) = control_input(2); // Joint 6
      uhat_update = (M_task_inv* J_inv.transpose() * control_input).value();

      // -----------------------------------------------------------------------
      // [6. 제어 입력 합산]
      // -----------------------------------------------------------------------
      Eigen::VectorXd tau_d(7);
      Eigen::VectorXd tau_d2(7);

      tau_d2 = tau_pd + tau_qp; // 최종 목표

      tau_d = tau_pd_total;  // pd만 실험(제자리 유지)

      // -----------------------------------------------------------------------
      // 터미널 상에 출력
      // -----------------------------------------------------------------------
      time_print_timer += duration.toSec(); // 지난 시간을 계속 더함
    if (time_print_timer >= 2.0) {
        std::cout << "================ [Time: " << time << "s] ================" << std::endl;
        
        // 1. 현재 입력 토크 (tau_cmd)
        std::cout << "tau_pd (Total): " << tau_pd_total.transpose() << std::endl;
        std::cout << "tau_qp (Total): " << tau_d2.transpose() << std::endl;

        // 2. 현재 Z축 위치 (제어가 잘 되는지 확인용)
        // std::cout << "Current Z Pos: " << target.position(2) << std::endl;

        // 타이머 초기화 (다시 0부터 셈)
        time_print_timer = 0.0;
    }

      
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
      // [7. Data Logging] (기존 데이터)
      // -----------------------------------------------------------------------
      myfile << time << " "
            << position[0] << " " << position[1] << " " << position[2] << " "
            << position_d << " "
            // << " " << position_d[1] << " " << position_d[2] << " "
            << current_velocity_display[0] << " " << current_velocity_display[1] << " " << current_velocity_display[2] << " "
            << orientation.x() << " " << orientation.y() << " " << orientation.z() << " "
            << orientation_d.x() << " " << orientation_d.y() << " " << orientation_d.z() << " "
            // << qpc_output_tau[0] << " " << qpc_output_tau[1] << " " << qpc_output_tau[2] << " "
            // << qpc_output_tau[3] << " " << qpc_output_tau[4] << " " << qpc_output_tau[5] << " " << qpc_output_tau[6]
            << tau_qp[0] << " " << tau_qp[1] << " " << tau_qp[2] << " "
            << tau_qp[3] << " " << tau_qp[4] << " " << tau_qp[5] << " " << tau_qp[6]  << "\n";

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

Eigen::VectorXd generalizedCrossProduct(const Eigen::Matrix<double, 1, 3>& matrix) {
  Eigen::VectorXd result(3);
  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix<double, 1, 1> subMatrix;
    int col = 0;
    for (int j = 0; j < 3; ++j) {
      if (i == j) continue;
      subMatrix.col(col++) = matrix.col(j);
    }
    result(i) = std::pow(-1, i) * subMatrix.determinant();
  }
  return result;
}

Eigen::Matrix<double, 3, 1> solveqpOASES(
      const Eigen::Matrix<double, 3, 3> &P, const Eigen::Matrix<double, 3, 1> &q,
      const Eigen::Matrix<double, 3, 1> &lb, const Eigen::Matrix<double, 3, 1> &ub) {
    
    qpOASES::real_t H[9];
    qpOASES::real_t g[3];
    qpOASES::real_t lb_in[3];
    qpOASES::real_t ub_in[3];

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            H[r*3 + c] = static_cast<qpOASES::real_t>(P(r,c));
        }
        g[r]  = static_cast<qpOASES::real_t>(q(r));
        lb_in[r] = static_cast<qpOASES::real_t>(lb(r));
        ub_in[r] = static_cast<qpOASES::real_t>(ub(r));
    }
    
    // 일반 QProblem 클래스 (변수 7개, 제약조건 0개)
    static qpOASES::QProblem qp_solver(3, 0);
    
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    options.enableFlippingBounds = qpOASES::BT_TRUE;
    qp_solver.setOptions(options);
    int nWSR = 150;
    
    // init(H, g, A, lb, ub, lbA, ubA, nWSR)
    qp_solver.init(H, g, nullptr, lb_in, ub_in, nullptr, nullptr, nWSR);
    qpOASES::real_t sol[3] = {0};
    qp_solver.getPrimalSolution(sol);
    
    Eigen::Matrix<double,3,1> x;
    for(int i=0; i<3; ++i) x(i) = sol[i];
    return x;
}