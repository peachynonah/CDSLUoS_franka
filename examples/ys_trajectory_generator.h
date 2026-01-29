#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

namespace ys_trajectory {

// 궤적 계산 결과를 담을 구조체 (3차원 고정)
struct TrajectoryState {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
};

class QuinticPolynomialTrajectory {
 public:
  QuinticPolynomialTrajectory() = default;
  ~QuinticPolynomialTrajectory() = default;

  /**
   * @brief 궤적 생성기 초기화 (3D Position Only)
   * @param start_pos 시작 위치 (x, y, z)
   * @param end_pos   목표 위치 (x, y, z)
   * @param time_duration 도달하는 데 걸리는 시간 (초)
   */
  void initTrajectory(const Eigen::Vector3d& start_pos, 
                      const Eigen::Vector3d& end_pos, 
                      double time_duration) {
    time_duration_ = time_duration;
    start_pos_ = start_pos;
    end_pos_ = end_pos;
    
    // 계수 행렬 초기화 (3x6)
    coefficients_.setZero();

    double T = time_duration;
    // T가 0이거나 매우 작을 경우 예외 처리 (나눗셈 방지)
    if (std::abs(T) < 1e-6) {
        // 시간 0이면 움직이지 않도록 설정
        coefficients_.col(0) = start_pos; // a0 = start_pos
        return;
    }

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    // Minimum Jerk Trajectory 계수 계산 (v0=a0=vf=af=0 가정)
    // i=0: x, i=1: y, i=2: z
    for (int i = 0; i < 3; ++i) {
      double x0 = start_pos(i);
      double xf = end_pos(i);

      coefficients_(i, 0) = x0;       // a0
      coefficients_(i, 1) = 0.0;      // a1
      coefficients_(i, 2) = 0.0;      // a2
      coefficients_(i, 3) = (10.0 * (xf - x0)) / T3; // a3
      coefficients_(i, 4) = (15.0 * (x0 - xf)) / T4; // a4
      coefficients_(i, 5) = (6.0 * (xf - x0)) / T5;  // a5
    }
  }

  /**
   * @brief 현재 시간(t)에 해당하는 상태 반환 (3D)
   */
  TrajectoryState computeState(double time) {
    TrajectoryState state;
    
    // 고정 크기이므로 setZero로 초기화
    state.position.setZero();
    state.velocity.setZero();
    state.acceleration.setZero();

    // 시간이 종료 시간을 넘어가면 마지막 상태 유지
    if (time >= time_duration_) {
      time = time_duration_;
      state.position = end_pos_;
      state.velocity.setZero();
      state.acceleration.setZero();
      return state;
    } else if (time < 0) {
      time = 0;
    }

    double t = time;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    // 3차원(x, y, z)에 대해 계산
    for (int i = 0; i < 3; ++i) {
      double a0 = coefficients_(i, 0);
      double a1 = coefficients_(i, 1);
      double a2 = coefficients_(i, 2);
      double a3 = coefficients_(i, 3);
      double a4 = coefficients_(i, 4);
      double a5 = coefficients_(i, 5);

      // Position
      state.position(i) = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
      
      // Velocity
      state.velocity(i) = a1 + 2.0*a2*t + 3.0*a3*t2 + 4.0*a4*t3 + 5.0*a5*t4;
      
      // Acceleration
      state.acceleration(i) = 2.0*a2 + 6.0*a3*t + 12.0*a4*t2 + 20.0*a5*t3;
    }

    return state;
  }

 private:
  double time_duration_;
  Eigen::Vector3d start_pos_;
  Eigen::Vector3d end_pos_;
  
  // 3행(x,y,z) x 6열(a0~a5) 행렬
  Eigen::Matrix<double, 3, 6> coefficients_;
};

} // namespace ys_trajectory