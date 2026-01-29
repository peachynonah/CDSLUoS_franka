// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

double getasafe(
            Eigen::Matrix<double, 2, 2> Q_matrix,
            Eigen::Matrix<double, 2, 2> P_matrix,
            Eigen::Matrix<double, 2, 1> state_error,
            double clbf_slope_l_,
            double unsafe_d_,
            double clbf_margin_delta_,
            double clbf_weight_theta_,
            double cart_pos_current
          ){
  // Q_matrix_Lyapunov_1  << 1.0, 0.3,
  //                         0.3, 1.0; //example
  
  // P_matrix_Lyapunov_1  << 5.75, 0.1,
  //                         0.1, 1.2; //example

  Eigen::Matrix<double, 2, 1> B_matrix;
  B_matrix << 0.0, 1.0; //example
  
  // Eigen::Matrix<double, 2, 1> err_1;
  // err_1 << cart_pos_err(0), cart_vel_err(0);

  double CLF_V_1 = 0.5 * (state_error.transpose() * P_matrix * state_error)(0,0);
  double temp_Qmat = (state_error.transpose() * Q_matrix * state_error)(0,0);

  // --- scalar version of CLBF computation ---
  double sigma = 1.0 / (1.0 + std::exp(clbf_slope_l_ * (cart_pos_current - unsafe_d_ - 0.5 * clbf_margin_delta_)));
  double nonlinear_weight = 1.0 + clbf_weight_theta_ * sigma;
  double CLBF_W = nonlinear_weight * CLF_V_1;
  double alpha = - nonlinear_weight * temp_Qmat
          - clbf_weight_theta_ * CLF_V_1 * (clbf_slope_l_ * sigma * (1.0 - sigma) * state_error(1));
  double beta = nonlinear_weight * (state_error.transpose() * P_matrix * B_matrix)(0,0);
  double beta_sq  = beta * beta;
  double beta_4th = beta_sq * beta_sq;
  double asafe = -((alpha + std::sqrt(alpha * alpha + 1.0 * beta_4th))/beta_sq) * beta;

  return asafe;
}

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  // const double translational_stiffness{50.0}; // original stiffness: 150
  const double translational_stiffness{50.0}; //
  
  const double rotational_stiffness{150.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);

  // damping.topLeftCorner(3, 3) << 0.5 * Eigen::MatrixXd::Identity(3, 3);

  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  // Initialize file_ hcpyon
  std::ofstream myfile; //
  // std::string base_dir = "/home/libfranka/hc_data/";
  // std::string filename = argv[2];
  // std::string full_path = base_dir + filename;
  myfile.open(argv[2], std::ios::out);
  
  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    //Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Vector3d init_position(initial_transform.translation());//hcpyon
    Eigen::Vector3d position_d = init_position; //hcpyon
    Eigen::Vector3d velocity_d = Eigen::Vector3d::Zero(); 
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    // Set collision behavior: from Cartesian pose controller
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    double time = 0.0; // hcpyon
    static double print_time = 0.0; // hcpyon
    int rt_violation_count = 0; // hcpyon
    static const double dt_threshold = 0.002;  // 2 ms

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      
      time += duration.toSec(); //hcpyon

      // ---- RT violation check ----
      if (duration.toSec() > dt_threshold) {
        rt_violation_count++;
        std::cout << "[RT VIOLATION] Δt = " << duration.toSec()
                  << " s | total count: " << rt_violation_count << std::endl;
      }

      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 49> mass_array = model.mass(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      std::array<double, 7> gravity_array = model.gravity(robot_state); // hcpyon
      
      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data()); //hcpyon
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data()); //hcpyon

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());
      Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;
      //Jacobian pseudoinverse
      Eigen::Matrix<double, 6, 6> lambda_inv = jacobian * mass.inverse() * jacobian.transpose();
      Eigen::Matrix<double, 6, 6> lambda = lambda_inv.inverse();
      Eigen::Matrix<double, 7, 6> Jbar = mass.inverse() * jacobian.transpose() * lambda;
      
      // y axis disturbance
      Eigen::Matrix<double, 6, 1> w_dist;
      w_dist.setZero();

      const double t0  = 2.0;
      const double dur = 0.1;
      const double Fy  = 3.0;

      double fmag = 0.0;
      if (time >= t0 && time <= t0 + dur) {
        double s = (time - t0) / dur;
        fmag = Fy * std::sin(M_PI * s);
      }

      w_dist(1) = fmag;

      //reference
      constexpr double kRadius = 0.3;
      constexpr double slower = 1.0;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * slower * time));
      double dx_trk = kRadius * std::sin(angle);
      double dz_trk = kRadius * (std::cos(angle) - 1);

      double angle_dot = (M_PI / 4) * (M_PI / 5.0 * slower * std::sin(M_PI / 5.0 * slower * time));
      double dx_vel_trk = kRadius * (std::cos(angle)) * angle_dot;
      double dz_vel_trk = kRadius * (- std::sin(angle)) * angle_dot;

      position_d = init_position + Eigen::Vector3d(dx_trk, 0.0, dz_trk); //; 
      velocity_d = Eigen::Vector3d(dx_vel_trk, 0.0, dz_vel_trk);

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error, error_vel;
      error.head(3) << position - position_d;
      error_vel << velocity;
      error_vel.head<3>() -= velocity_d;
      
      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }

      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);
      
      // compute control
      Eigen::VectorXd tau_task(7), tau_temp(7), tau_d(7), force_FL(6), force_CLBF(6), asafe(6);

      // // Spring damper system with damping ratio=1
      // tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      // tau_d << tau_task + coriolis;

      Eigen::Matrix<double, 2, 2> Q_matrix_z, P_matrix_z;
      Q_matrix_z << 1.0, 0.3,
                    0.3, 1.0; 
      P_matrix_z << 5.75, 0.1,
                     0.1, 1.2;
      Eigen::Matrix<double, 2, 1> state_error_z;
      state_error_z << error(2),
                       error_vel(2);
      
      double clbf_slope_l_z = 3.0;
      // double unsafe_d_z = 0.35;
      double unsafe_d_z = -0.1;
      double clbf_margin_delta_z = 0.1;
      double clbf_weight_theta_z = 5.0;
      // double cart_pos_current_z = position(2);
      double cart_pos_current_z = error(2);
      
      // hcpyon -----------------------------
      print_time += duration.toSec();

      if (print_time > 0.5) {
        // std::cout << "Time : ";
        // std::cout << time << " ";
        // std::cout << std::endl;

          std::cout << "[error] ["
            << error(0) << ", "
            << error(1) << ", "
            << error(2) << " | "
            << error(3) << ", "
            << error(4) << ", "
            << error(5) << "]"
            << std::endl;

            std::cout << "||F_trans||=" << force_FL.head<3>().norm()
            << " ||F_rot||="  << force_FL.tail<3>().norm() << "\n";

        print_time = 0.0;
      }
      // -----------------------------

      double asafe_z = getasafe(Q_matrix_z, P_matrix_z, state_error_z, clbf_slope_l_z, unsafe_d_z, clbf_margin_delta_z, clbf_weight_theta_z, cart_pos_current_z);
      // if (asafe_z < 0.0)
      // asafe_z = 0.0;
      
      asafe.setZero();     
      asafe(2) = asafe_z;
      
      // tau_temp << mass * Jbar * (-stiffness * error + asafe);
      force_FL << lambda * (-stiffness * error - damping * error_vel)  + Jbar.transpose() * coriolis;
      force_CLBF << lambda * asafe;

      // Clip force_CLBF
      for (int i = 0; i < force_CLBF.size(); ++i) {
          // double max_val = 1.5 * std::abs(force_FL(i));
          double max_val = 8.0;
          if (force_CLBF(i) >  max_val) force_CLBF(i) =  max_val;
          if (force_CLBF(i) < -max_val) force_CLBF(i) = -max_val;
      }
      

      // tau_d << jacobian.transpose() * (force_FL);
      tau_d << jacobian.transpose() * (force_FL + w_dist); 
      // tau_d << jacobian.transpose() * (force_FL + force_CLBF);

      // File to store the states and force
      for (int i = 0; i < 16; i++){
          myfile << robot_state.O_T_EE[i] << " "; // Homogeneous Transform of End Effector (4 by 4 Matrix)
      }
      for (int i = 0; i < 6; i++) {
          myfile << force_FL[i] << " ";
      }
      for (int i = 0; i < 6; i++) {
          myfile << force_CLBF[i] << " ";
      }
      for (int i = 0; i < 3; i++) {
          myfile << position_d[i] << " ";
      }
      for (int i = 0; i < 6; i++) {
          myfile << velocity[i] << " ";
      }
      for (int i = 0; i < 3; i++) {
          myfile << velocity_d[i] << " ";
      }
      myfile << '\n'; //hcpyon

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
