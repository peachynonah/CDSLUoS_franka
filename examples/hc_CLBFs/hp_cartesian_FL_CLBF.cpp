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

#include "sigmoid_clbf/sigmoid_clbf.h"
#include "ref_gen/ReferenceGenerator.h"

ReferenceGenerator ref_crt_x;
ReferenceGenerator ref_crt_y;
ReferenceGenerator ref_crt_z;

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  const double translational_stiffness{50.0}; // original stiffness: 50 ~ 150
  const double rotational_stiffness{10.0};


  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

  // Initialize file
  std::ofstream myfile;
  myfile.open(argv[2], std::ios::out);
  
  // File header indices
  myfile <<
  "O_T_EE_00 O_T_EE_01 O_T_EE_02 O_T_EE_03 "
  "O_T_EE_04 O_T_EE_05 O_T_EE_06 O_T_EE_07 "
  "O_T_EE_08 O_T_EE_09 O_T_EE_10 O_T_EE_11 "
  "O_T_EE_12 O_T_EE_13 O_T_EE_14 O_T_EE_15 "

  "F_FL_ana_x F_FL_ana_y F_FL_ana_z F_FL_ana_rx F_FL_ana_ry F_FL_ana_rz "
  
  "pos_des_x pos_des_y pos_des_z "
  "vel_des_x vel_des_y vel_des_z "
  
  "pos_x pos_y pos_z "
  
  "bodyRPY_des_R bodyRPY_des_P bodyRPY_des_Y "
  "bodyRPY_R bodyRPY_P bodyRPY_Y "

  "error_ana_x error_ana_y error_ana_z error_ana_rx error_ana_ry error_ana_rz "
    
  "\n"; // if output data of csv is modified, then this part should also be modified. 

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

        // First move the robot to a suitable joint configuration
    // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, -M_PI_4, M_PI_2, M_PI_4}};

    MotionGenerator motion_generator(0.15, q_goal);
    std::cout << "WARNING: This work will move the robot! " << std::endl
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to move robot to initial joint configuration..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d init_position(initial_transform.translation());
    // Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Vector3d init_ori_XYZ = initial_transform.linear().eulerAngles(0,1,2);
    //debugMar11
    Eigen::Vector3d init_ori_ZYX = initial_transform.linear().eulerAngles(2,1,0);

    // Set collision behavior: from Cartesian pose controller
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    double time = 0.0; 
    static double print_time = 0.0; 
    // int rt_violation_count = 0;
    // static const double dt_threshold = 0.002;  // 2 ms

    //reference generator_predefined values
    double time_ref_start_ = 0.0;
    double time_ref_fin_ = 10.0;
    constexpr double kRadius = 0.3;

    // std::array<double, 3> des_pos_mov = {kRadius, 0, -kRadius};
    std::array<double, 3> des_pos_mov = {0.2, 0, 0};

    std::array<double, 3> start_state_x = {init_position[0], 0.0, 0.0};
    std::array<double, 3> final_state_x = {init_position[0] + des_pos_mov[0], 0.0, 0.0};
    
    std::array<double, 3> start_state_y = {init_position[1], 0.0, 0.0};
    std::array<double, 3> final_state_y = {init_position[1] + des_pos_mov[1], 0.0, 0.0};
    
    std::array<double, 3> start_state_z = {init_position[2], 0.0, 0.0};
    std::array<double, 3> final_state_z = {init_position[2] + des_pos_mov[2], 0.0, 0.0};

    ref_crt_x.computeAlphaCoeffs(time_ref_start_, time_ref_fin_, start_state_x, final_state_x);
    ref_crt_y.computeAlphaCoeffs(time_ref_start_, time_ref_fin_, start_state_y, final_state_y);
    ref_crt_z.computeAlphaCoeffs(time_ref_start_, time_ref_fin_, start_state_z, final_state_z);

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      
      time += duration.toSec();

      // // ---- RT violation check ----
      // if (duration.toSec() > dt_threshold) {
      //   rt_violation_count++;
      //   std::cout << "[RT VIOLATION] Δt = " << duration.toSec()
      //             << " s | total count: " << rt_violation_count << std::endl;
      // }

      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 49> mass_array = model.mass(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      std::array<double, 7> gravity_array = model.gravity(robot_state);
      
      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      Eigen::Matrix<double, 6, 1> crt_vel_eig = jacobian * dq; 

      std::array<double, 6> crt_vel_std;
      for (int i = 0; i < 6; ++i) {crt_vel_std[i] = crt_vel_eig(i);}

      // == Part 1. Reference generation
      std::array<double, 3> pos_des, pose_dot_des, pose_ddot_des;

      pos_des[0] = ref_crt_x.get_position(time, position[0]);
      pose_dot_des[0] = ref_crt_x.get_velocity(time, crt_vel_std[0]);
      pose_ddot_des[0] = ref_crt_x.get_acceleration(time, 0.0);

      pos_des[1] = ref_crt_y.get_position(time, position[1]);
      pose_dot_des[1] = ref_crt_y.get_velocity(time, crt_vel_std[1]);
      pose_ddot_des[1] = ref_crt_y.get_acceleration(time, 0.0);

      pos_des[2] = ref_crt_z.get_position(time, position[2]);
      pose_dot_des[2] = ref_crt_z.get_velocity(time, crt_vel_std[2]);
      pose_ddot_des[2] = ref_crt_z.get_acceleration(time, 0.0);

      Eigen::Vector3d pos_des_eig, pose_dot_des_eig, pose_ddot_des_eig;
      pos_des_eig << pos_des[0], pos_des[1], pos_des[2];
      pose_dot_des_eig << pose_dot_des[0], pose_dot_des[1], pose_dot_des[2];
      pose_ddot_des_eig << pose_ddot_des[0], pose_ddot_des[1], pose_ddot_des[2];

      Eigen::Vector3d ori_XYZ_des;
      ori_XYZ_des << init_ori_XYZ[0], init_ori_XYZ[1], init_ori_XYZ[2];

      Eigen::Vector3d ori_ZYX_des;
      ori_ZYX_des << init_ori_ZYX[0], init_ori_ZYX[1], init_ori_ZYX[2];

      // Eigen::Matrix<double, 6, 1> error, error_dot;
      // error.head(3) << position - pos_des_eig;
      // error_dot.head(3) << crt_vel_eig.head(3) - pose_dot_des_eig;
      // error_dot.tail(3) << crt_vel_eig.tail(3);

      Eigen::Matrix<double, 6, 1> acc_des;
      acc_des.head(3) << pose_ddot_des_eig;
      acc_des.tail(3).setZero();

      // orientation error
      // if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
      //   orientation.coeffs() << -orientation.coeffs();
      // }
      // // "difference" quaternion
      // Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // // Transform to base frame
      // error.tail(3) << -transform.linear() * error.tail(3);

      //Geometric Jacobian pseudoinverse
      // Eigen::Matrix<double, 6, 6> lambda_inv = jacobian * mass.inverse() * jacobian.transpose();
      // Eigen::Matrix<double, 6, 6> lambda = lambda_inv.inverse();
      // Eigen::Matrix<double, 7, 6> Jbar = mass.inverse() * jacobian.transpose() * lambda;

      // compute control
      // Eigen::VectorXd force_FL(6);

      // force_FL << lambda * (-stiffness * error - damping * error_dot) + Jbar.transpose() * coriolis;


      ///////////////////// ****************** == Part 2. Analytic Jacobian ******************  ////////////
      
      //=== Part 2-1. Trial 1: Analytic Jacobian (X-Y-Z)
      // Eigen::Vector3d base_XYZ = transform.linear().eulerAngles(0,1,2);

      // double phi   = base_XYZ[0];  
      // double theta = base_XYZ[1];  
      // double psi   = base_XYZ[2]; 

      // Eigen::Matrix3d T_inv;
      // T_inv <<1,  sin(phi)*tan(theta),  cos(phi)*tan(theta),
      //         0,  cos(phi),            -sin(phi),
      //         0,  sin(phi)/cos(theta),  cos(phi)/cos(theta);

      // Eigen::Matrix<double, 6, 6> Ta_inv = Eigen::Matrix<double, 6, 6>::Identity();
      // Ta_inv.bottomRightCorner(3, 3) = T_inv;

      // Eigen::Matrix<double, 6, 7> jacobian_ana = Ta_inv * jacobian;
      // // == Original Part 2. wrap 

      // == debugMar11
      // == Part 2-2. Trial 2: Analytic Jacobian (Z-Y-X)
      Eigen::Vector3d base_ZYX = transform.linear().eulerAngles(2, 1, 0);
      double alpha = base_ZYX[0];  // yaw   (Z)
      double beta  = base_ZYX[1];  // pitch (Y)
      double gamma = base_ZYX[2];  // roll  (X)
      // T: omega_base = T * eta_dot  (Siciliano Handbook eq. 1.5)
      Eigen::Matrix3d Transform_matrix_euler;
      Transform_matrix_euler <<            -sin(beta),           0, 1,
                                 cos(beta)*sin(gamma),  cos(gamma), 0,
                                 cos(beta)*cos(gamma),  -sin(beta), 0;

      Eigen::Matrix<double, 6, 6> Transform_matrix_large = Eigen::Matrix<double, 6, 6>::Identity();
      Transform_matrix_large.bottomRightCorner(3, 3) = Transform_matrix_euler;                    
      Eigen::Matrix<double, 6, 7> jacobian_ana = Transform_matrix_large * jacobian;
      // == Part 2-1 END

      Eigen::Matrix<double, 6, 6> lambda_inv_ana = jacobian_ana * mass.inverse() * jacobian_ana.transpose();
      Eigen::Matrix<double, 6, 6> lambda_ana     = lambda_inv_ana.inverse();
      Eigen::Matrix<double, 7, 6> Jbar_ana       = mass.inverse() * jacobian_ana.transpose() * lambda_ana;

      // DEBUG 1. effective mass matrix의 다른 계산방법 (lambda_ana와 동일하게 나옴)
      Eigen::Matrix<double, 7, 6> jac_W_dag = mass.inverse() * jacobian_ana.transpose()*(jacobian_ana * mass.inverse() * jacobian_ana.transpose()).inverse();
      Eigen::Matrix<double, 6, 6> Mass_t_in_paper = jac_W_dag.transpose() * mass * jac_W_dag;

      print_time += duration.toSec();
      if (print_time >= 0.5) {
        std::cout << "lambda_ana:\n" << lambda_ana << std::endl;
        std::cout << "Mass_t_in_paper:\n" << Mass_t_in_paper << std::endl;
        print_time = 0.0;
      }

      Eigen::Matrix<double, 6, 1> crt_vel_ana = jacobian_ana * dq;

      Eigen::Matrix<double, 6, 1> error_ana, error_dot_ana;
      error_ana.head(3)     << position - pos_des_eig;
      // error_ana.tail(3)     << base_XYZ - ori_XYZ_des; // X-Y-Z Euler angle error
      error_ana.tail(3)     << base_ZYX - ori_ZYX_des;

      error_dot_ana.head(3) << crt_vel_ana.head(3) - pose_dot_des_eig;
      error_dot_ana.tail(3) << crt_vel_ana.tail(3); // desired Euler rate = 0

      Eigen::Matrix<double, 6, 1> force_FL_ana;
      force_FL_ana << lambda_ana * (acc_des - stiffness * error_ana - damping * error_dot_ana)
                    + Jbar_ana.transpose() * coriolis; // + lambda_ana * Jdot * qdot;
      // ===== End Analytic =====

      // === Part 3. Safety-related forces ===

      // == Part 3-1. Virtual Disturbance generation
      Eigen::Matrix<double, 6, 1> force_virt_dstrb;
      force_virt_dstrb.setZero();

      double amp_rx = 0.05; double freq_rx = 0.5;
      force_virt_dstrb(3) = amp_rx * std::sin(2.0 * M_PI * freq_rx * time); 


    //   // == Part 3-2. CLBF add-on input ==

    //   Eigen::VectorXd force_CLBF(6), asafe(6);
    //   Eigen::Matrix<double, 2, 2> Q_matrix_z, P_matrix_z;
    //   Q_matrix_z << 1.0, 0.3,
    //                 0.3, 1.0; 
    //   P_matrix_z << 5.75, 0.1,
    //                  0.1, 1.2;
    //   Eigen::Matrix<double, 2, 1> state_error_z;
    //   state_error_z << error(2),
    //                    error_dot(2);
    //   double clbf_slope_l_z = 3.0; double unsafe_d_z = -0.1;
    //   double clbf_margin_delta_z = 0.1; double clbf_weight_theta_z = 5.0;
    //   double cart_pos_current_z = error(2);
    //   print_time += duration.toSec();
    //   if (print_time > 0.5) {
    //     std::cout << "=== MODEL VALUES ===" << std::endl;
    //     std::cout << "Time : "; std::cout << time << " "; std::cout << std::endl;
    //     std::cout << "Z axis error : "; std::cout << error(2) << " "; std::cout << std::endl;
    //     print_time = 0.0;
    //   }
    //   // asafe
    //   double asafe_z = getasafe(Q_matrix_z, P_matrix_z, state_error_z, clbf_slope_l_z, unsafe_d_z, clbf_margin_delta_z, clbf_weight_theta_z, cart_pos_current_z);
    //   asafe.setZero();     
    //   asafe(2) = asafe_z;
    //   force_CLBF << lambda * asafe;
    //   // Clip force_CLBF
    //   for (int i = 0; i < force_CLBF.size(); ++i) {
    //       // double max_val = 1.5 * std::abs(force_FL(i));
    //       double max_val = 8.0;
    //       if (force_CLBF(i) >  max_val) force_CLBF(i) =  max_val;
    //       if (force_CLBF(i) < -max_val) force_CLBF(i) = -max_val;
    //   }
      
    // Part 4. ==== final tau calculation
      // Eigen::Matrix<double, 7, 1> tau_ana = jacobian_ana.transpose() * (force_FL_ana + force_virt_dstrb);
      Eigen::Matrix<double, 7, 1> tau_ana = jacobian_ana.transpose() * (force_FL_ana);

      Eigen::VectorXd tau_d(7);
      tau_d << tau_ana;

    // Part 5. === File to store the states and force
      for (int i = 0; i < 16; i++){myfile << robot_state.O_T_EE[i] << " ";}

      for (int i = 0; i < 6; i++) {myfile << force_FL_ana[i] << " ";}
            
      for (int i = 0; i < 3; i++) {myfile << pos_des[i] << " ";}
      for (int i = 0; i < 3; i++) {myfile << pose_dot_des[i] << " ";}
      
      for (int i = 0; i < 3; i++) {myfile << position[i] << " ";}
      
      // for (int i = 0; i < 3; i++) {myfile << ori_XYZ_des[i] << " ";}
      // for (int i = 0; i < 3; i++) {myfile << base_XYZ[i] << " ";}

      for (int i = 0; i < 3; i++) {myfile << ori_ZYX_des[i] << " ";}
      for (int i = 0; i < 3; i++) {myfile << base_ZYX[i] << " ";}
      
      for (int i = 0; i < 6; i++) {myfile << error_ana[i] << " ";}
      
      myfile << '\n';

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };

    // Part 6. === start real-time control loop
    std::cout << "WARNING: Make sure you have the user stop at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;

    std::cin.ignore();
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
