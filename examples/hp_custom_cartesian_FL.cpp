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

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  // const double translational_stiffness{50.0}; // original stiffness: 150
  const double translational_stiffness{5.0}; // stiffness_reg
  
  const double rotational_stiffness{1.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 0.10 * 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  // Initialize file_ hcpyon
  std::ofstream myfile; //
  // std::string base_dir = "/home/libfranka/hc_data/";
  // std::string filename = argv[2];
  // std::string full_path = base_dir + filename;
  myfile.open(argv[2], std::ios::out); // hcpyon
  
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
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    // // set collision behavior: impedance controller collision behavior
    // robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // Set collision behavior: cartesian pose controller
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

      // File to store the states
      for (int i = 0; i < 16; i++) //hcpyon
      {
          myfile << robot_state.O_T_EE[i] << " ";
      }
      myfile << '\n'; //hcpyon

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
      std::array<double, 6> CLBF_asafe = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // hcpyon

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

      //Jacobian pseudoinverse
      Eigen::Matrix<double, 6, 6> lambda_inv = jacobian * mass.inverse() * jacobian.transpose();
      Eigen::Matrix<double, 6, 6> lambda = lambda_inv.inverse();
      Eigen::Matrix<double, 7, 6> Jbar = mass.inverse() * jacobian.transpose() * lambda;
      Eigen::Matrix<double,6,1> asafe;
      for (int i = 0; i < 6; ++i) asafe(i) = CLBF_asafe[i];

      // // hcpyon --- (A) 시간 함수 기반 레퍼런스 생성 (원형 궤적 일부) ---
      // constexpr double kRadius = 0.3; 
      // const double slow = 0.8; // 0.5배 속도 = 2배 느리게
      // const double angle  = M_PI/4.0 * (1.0 - std::cos(M_PI/5.0 * time * slow));
      // const double dx     = kRadius * std::sin(angle);
      // const double dz     = kRadius * (std::cos(angle) - 1.0); //hcpyon

      // position_d = init_position + Eigen::Vector3d(dx, 0.0, dz); //; 

      // hcpyon --- (B) regulation problem goal point
      const double dx_reg     = 0.2;
      const double dz_reg     = -0.2; //hcpyon

      position_d = init_position + Eigen::Vector3d(dx_reg, 0.0, dz_reg); //; 

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;

      // hcpyon -----------------------------
      print_time += duration.toSec();

      if (print_time > 0.5) {
        std::cout << "=== MODEL VALUES ===" << std::endl;

        std::cout << "Time : ";
        std::cout << time << " ";
        std::cout << std::endl;

        print_time = 0.0;
      }
      // -----------------------------

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
      Eigen::VectorXd tau_task(7), tau_temp(7), tau_d(7);

      // // Spring damper system with damping ratio=1
      // tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      // tau_d << tau_task + coriolis;

      // FL controller
      // tau_temp << mass * Jbar * (-stiffness * error + asafe);
      tau_temp << jacobian.transpose() * lambda * (-stiffness * error - damping * (jacobian * dq) + asafe);
      tau_d << tau_temp + coriolis;

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
