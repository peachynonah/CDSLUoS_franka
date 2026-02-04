#pragma once
#include <array>
#include <Eigen/Dense>

#define reference_uprising 0
#define reference_downfalling 1

class ReferenceGenerator {

    public:
        std::array<double, 6> alpha_coeffs; // 다항식 계수들
        std::array<double, 3> start_state; // 시작 상태 [위치, 속도, 가속도]
        std::array<double, 3> final_state; // 최종 상태 [위치, 속도, 가속도]
        double time_ref_start; // 궤적 시작 시간
        double time_ref_fin; // 궤적 종료 시간
        
    public:
        // 생성자
        ReferenceGenerator();
        void computeAlphaCoeffs(double time_ref_start_,
                                double time_ref_fin_,
                                std::array<double, 3> start_state_,
                                std::array<double, 3> final_state_
                                /*std::array<double, 6> alpha_coeffs_*/);

        // 특정 시간 t에 대한 목표 위치를 계산
        double get_position(double current_time_,
                            double current_joint_position_);
        
        // 특정 시간 t에 대한 목표 속도를 계산
        double get_velocity(double current_time_,
                            double current_joint_velocity_);
        
        // 특정 시간 t에 대한 목표 가속도를 계산
        double get_acceleration(double current_time_,
                                double current_joint_acceleration_);
};