#include "ReferenceGenerator.h"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <array>
#include <Eigen/Dense>

// 생성자
ReferenceGenerator::ReferenceGenerator()
   : alpha_coeffs{0, 0, 0, 0, 0, 0},
     start_state{0, 0, 0},
     final_state{0, 0, 0},
     time_ref_start(0.0),
     time_ref_fin(0.0)
    {

    }

// sudo mv /home/kiro/workspace/CnSim_linux/cdsl_data.csv /home/kiro/CDSL/CDSL_f
// ire_monitor_control/csv_datas/trial3/cdsl_data_t3_12.csv

void ReferenceGenerator::computeAlphaCoeffs(double time_ref_start_,
                                            double time_ref_fin_,
                                            std::array<double, 3> start_state_,
                                            std::array<double, 3> final_state_
                                            /*std::array<double, 6> alpha_coeffs_*/) {
// 변수 선언
this->time_ref_start = time_ref_start_;
this->time_ref_fin = time_ref_fin_;
this->start_state = start_state_;
this->final_state = final_state_;

double tsf = this->time_ref_fin - this-> time_ref_start;
Eigen::Matrix<double, 6, 6> PHI;

PHI <<                    0,                     0,                    0,                 0,           0,             1,
                          0,                     0,                    0,                 0,           1,             0,
                          0,                     0,                    0,                 2,           0,             0,
           std::pow(tsf, 5),      std::pow(tsf, 4),     std::pow(tsf, 3),  std::pow(tsf, 2),         tsf,             1,
       5 * std::pow(tsf, 4),  4 * std::pow(tsf, 3), 3 * std::pow(tsf, 2),           2 * tsf,           1,             0,
      20 * std::pow(tsf, 3), 12 * std::pow(tsf, 2),              6 * tsf,                 2,           0,             0;

std::cout << "Matrix PHI:\n" << PHI << std::endl << std::endl;

Eigen::Matrix<double, 6, 6> PHI_inverse = PHI.inverse();
std::cout << "PHI Inverse:\n" << PHI_inverse << std::endl << std::endl;

Eigen::Matrix<double, 6, 1> THETA_star;
    THETA_star << start_state[0],
                  start_state[1],
                  start_state[2],
                  final_state[0],
                  final_state[1],
                  final_state[2];

Eigen::Matrix<double, 6, 1> alpha_eigen;
alpha_eigen = PHI_inverse * THETA_star;         
std::cout << "Result alpha:\n" << alpha_eigen << std::endl;

for (int i = 0; i < 6; ++i) {
    this->alpha_coeffs[5 - i] = alpha_eigen(i);
}
//alpha 계산...은.. 일단 외부에서 계산해서 넣어주기로...
//  this->alpha_coeffs = alpha_coeffs_;
}


// 1. desired reference position at current time
double ReferenceGenerator::get_position(double current_time_,
                                        double current_joint_position_) {

    // 궤적 시작 전에는 초기 위치를 반환
    if (current_time_ < this->time_ref_start) {
        return current_joint_position_;
    }

    else if (current_time_ >= this->time_ref_fin) {
        return final_state[0];  // 최종 위치 반환
    }

        // 궤적 구간 내에서는 다항식 계산
    else{
    double t_diff = current_time_ - this->time_ref_start;
    // printf("inner loop reference generator, time difference is %f\n", t_diff);  
    return alpha_coeffs[5] * std::pow(t_diff, 5) +
           alpha_coeffs[4] * std::pow(t_diff, 4) +
           alpha_coeffs[3] * std::pow(t_diff, 3) +
           alpha_coeffs[2] * std::pow(t_diff, 2) +
           alpha_coeffs[1] * t_diff +
           alpha_coeffs[0];
    }
}

// 2. desired reference velocity at current time
double ReferenceGenerator::get_velocity(double current_time_,
                                        double current_joint_velocity_) {
    if (current_time_ < this->time_ref_start) {
        return current_joint_velocity_;
    }

    else if (current_time_ >= this->time_ref_fin) {
    return final_state[1];// 최종 속도 반환
    }

    else{
    double t_diff = current_time_ - this->time_ref_start;
    return 5 * alpha_coeffs[5] * std::pow(t_diff, 4) +
           4 * alpha_coeffs[4] * std::pow(t_diff, 3) +
           3 * alpha_coeffs[3] * std::pow(t_diff, 2) +
           2 * alpha_coeffs[2] * t_diff +
           alpha_coeffs[1];
    }
}

// 3. desired reference accleration at current time
double ReferenceGenerator::get_acceleration(double current_time_,
                                            double current_joint_acceleration_) {
    if (current_time_ < this->time_ref_start){
        return current_joint_acceleration_;
    }
    
    else if (current_time_ >= this->time_ref_fin) {
        return final_state[2]; // 최종 가속도 반환
    }
    
    else{
    double t_diff = current_time_ - this->time_ref_start;
    return 20 * alpha_coeffs[5] * std::pow(t_diff, 3) +
           12 * alpha_coeffs[4] * std::pow(t_diff, 2) +
           6 * alpha_coeffs[3] * t_diff +
           2 * alpha_coeffs[2];
    }
}