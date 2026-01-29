#pragma once

#include <Eigen/Dense>
// #include <eigen3/Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ys_trajectory_lspb {

// ---------------------------
// 결과 구조체 (3D 전용)
// ---------------------------
struct TrajectoryState3D {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

// ---------------------------
// 내부: 축(1-DOF) LSPB 파라미터
// ---------------------------
struct LspbAxisParams {
  double x0 = 0.0;  // 시작 위치
  double x1 = 0.0;  // 목표 위치

  double acc = 0.0; //제한 가속도
  double vel = 0.0; //제한 속도

  double ta  = 0.0;   // 가속 구간 시간
  double tf  = 0.0;   // 총 시간
  int phase  = 1;     // 1: trapezoid, 2: triangle

  // 부호(방향)
  double sgn = 1.0;   // +1 or -1
};

inline double sgn(double v) { return (v >= 0.0) ? 1.0 : -1.0; }

// MATLAB 코드와 동일한 논리로 "주어진 acc, vel 제약에서 최소 tf" 계산
inline LspbAxisParams compute_min_time_1d(double x0, double x1, double acc_max, double vel_max) {
  if (acc_max <= 0.0 || vel_max <= 0.0) {
    throw std::invalid_argument("acc_max and vel_max must be > 0");
  }

  LspbAxisParams p;
  p.x0 = x0;
  p.x1 = x1;
  p.acc = acc_max;
  p.vel = vel_max;

  const double dx = x1 - x0;
  const double D  = std::abs(dx);
  p.sgn = sgn(dx);

  if (D == 0.0) {
    // 이동 없음
    p.ta = 0.0;
    p.tf = 0.0;
    p.phase = 1;
    return p;
  }

  // MATLAB: ta = vel/acc
  double ta = vel_max / acc_max;

  // MATLAB(mode==0): tf = abs(x1-x0)/vel + ta
  double tf = D / vel_max + ta;

  // MATLAB: if tf/2 >= ta => phase 1 else phase 2
  if (tf / 2.0 >= ta) {
    p.phase = 1;
    // MATLAB: vel = (x1-x0)/(tf-ta), acc = vel/ta
    // 여기서 (x1-x0)는 부호 포함. 우리는 크기만 관리하고, 나중에 sgn을 곱함.
    const double vel = D / (tf - ta);
    const double acc = vel / ta;
    p.ta = ta;
    p.tf = tf;
    p.vel = vel;
    p.acc = acc;
  } else {
    // phase 2 (triangle)
    p.phase = 2;
    // MATLAB: ta = tf/2; tf = sqrt(4*D/acc)
    tf = std::sqrt(4.0 * D / acc_max);
    ta = tf / 2.0;
    // MATLAB: acc = 4*(x1-x0)/(tf*tf); vel = acc*ta
    const double acc = 4.0 * D / (tf * tf);
    const double vel = acc * ta;

    p.ta = ta;
    p.tf = tf;
    p.acc = acc;
    p.vel = vel;
  }

  return p;
}

// MATLAB 코드의 mode==1(tf 고정) 케이스 느낌: "주어진 tf로 LSPB 파라미터 재계산"
// - vel_max/acc_max는 "상한"으로 쓰고, 필요하면 triangle로 전환
inline LspbAxisParams compute_with_fixed_tf_1d(double x0, double x1,
                                               double tf,
                                               double acc_max, double vel_max) {
  if (tf < 0.0) throw std::invalid_argument("tf must be >= 0");
  if (acc_max <= 0.0 || vel_max <= 0.0) {
    throw std::invalid_argument("acc_max and vel_max must be > 0");
  }

  LspbAxisParams p;
  p.x0 = x0;
  p.x1 = x1;
  p.tf = tf;
  p.sgn = sgn(x1 - x0);

  const double dx = x1 - x0;
  const double D  = std::abs(dx);

  if (D == 0.0 || tf == 0.0) {
    p.ta = 0.0;
    p.phase = 1;
    p.acc = 0.0;
    p.vel = 0.0;
    return p;
  }

  // 우선 MATLAB처럼 ta = vel/acc로 시작하지만, 우리는 tf에 맞춰야 하므로 조건을 다시 맞춤
  double ta = vel_max / acc_max;

  // MATLAB: if tf/2 >= ta => phase 1 else phase 2 (triangle)
  if (tf / 2.0 >= ta) {
    p.phase = 1;

    // MATLAB: vel = (x1-x0)/(tf-ta), acc = vel/ta
    // 단, vel/acc가 상한을 넘지 않도록 ta를 조정할 수 있는데
    // 여기서는 "tf 고정 + vel_max/acc_max 상한"을 만족시키는 가장 단순한 형태로:
    double vel = D / (tf - ta);
    double acc = vel / ta;

    // 혹시 vel이 vel_max를 넘으면 => ta를 늘려 vel을 줄이기 (tf/2 이내)
    if (vel > vel_max) {
      // vel = D/(tf-ta) <= vel_max  => ta <= tf - D/vel_max
      const double ta_upper = tf - D / vel_max;

      ta = std::min(std::max(ta_upper, 0.0), tf / 2.0);
      vel = D / (tf - ta);
      acc = (ta > 0.0) ? (vel / ta) : acc_max;
    }

    // 혹시 acc가 acc_max를 넘으면 => triangle로 전환하는 게 안전
    if (acc > acc_max || ta <= 0.0) {
      p.phase = 2;
      ta = tf / 2.0;
      const double acc2 = 4.0 * D / (tf * tf);
      const double vel2 = acc2 * ta;
      p.ta = ta;
      p.acc = acc2;
      p.vel = vel2;
      return p;
    }

    p.ta = ta;
    p.vel = vel;
    p.acc = acc;
    return p;

  } else {
    // triangle 고정
    p.phase = 2;
    ta = tf / 2.0;
    const double acc = 4.0 * D / (tf * tf);
    const double vel = acc * ta;
    p.ta = ta;
    p.acc = acc;
    p.vel = vel;
    return p;
  }
}

// 단일 축에서 시간 t에 대한 (pos, vel, acc) 계산 (MATLAB 식 그대로)
inline void eval_axis(const LspbAxisParams& p, double t, double& x, double& xd, double& xdd) {
  if (t < 0.0) t = 0.0;
  if (t > p.tf) t = p.tf;

  // 이동없음
  if (p.tf == 0.0) {
    x   = p.x1;
    xd  = 0.0;
    xdd = 0.0;
    return;
  }

  const double a = p.acc * p.sgn;  // 부호 적용
  const double v = p.vel * p.sgn;

  if (p.phase == 1) {
    // trapezoid
    if (t < p.ta) {
      xdd = a;
      xd  = a * t;
      x   = p.x0 + (a / 2.0) * t * t;
    } else if (t < p.tf - p.ta) {
      xdd = 0.0;
      xd  = v;
      x   = p.x0 + (v / 2.0) * p.ta + v * (t - p.ta);
    } else {
      xdd = -a;
      xd  = -a * (t - p.tf);
      x   = p.x1 - (a / 2.0) * (t - p.tf) * (t - p.tf);
    }
  } else {
    // triangle
    const double ta = p.tf / 2.0;
    if (t < ta) {
      xdd = a;
      xd  = a * t;
      x   = p.x0 + (a / 2.0) * t * t;
    } else {
      xdd = -a;
      xd  = -a * (t - p.tf);
      x   = p.x1 - (a / 2.0) * (t - p.tf) * (t - p.tf);
    }
  }
}

// ---------------------------
// 3D LSPB Trajectory (x,y,z)
// ---------------------------
class LspbTrajectory3D {
 public:
  LspbTrajectory3D() = default;

  // mode=0: acc/vel 제한으로 최소시간 계산 후 3축 동기화
  // mode=1: 사용자가 tf를 지정(고정)하고 그 tf에 맞춰 3축 파라미터 재계산
  void init(const Eigen::Vector3d& start_xyz,
            const Eigen::Vector3d& goal_xyz,
            double acc_max,
            double vel_max,
            int mode = 0,
            double tf_fixed = 0.0) {
    if (acc_max <= 0.0 || vel_max <= 0.0) {
      throw std::invalid_argument("acc_max and vel_max must be > 0");
    }
    mode_ = mode;

    // 1) 축별 최소시간 계산
    if (mode_ == 0) {
      ax_[0] = compute_min_time_1d(start_xyz.x(), goal_xyz.x(), acc_max, vel_max);
      ax_[1] = compute_min_time_1d(start_xyz.y(), goal_xyz.y(), acc_max, vel_max);
      ax_[2] = compute_min_time_1d(start_xyz.z(), goal_xyz.z(), acc_max, vel_max);

      // 2) 3축 동기화: 가장 긴 tf를 전체 tf로
      tf_ = std::max({ax_[0].tf, ax_[1].tf, ax_[2].tf});

      // 3) 고정 tf로 각 축 파라미터 재산출(각 축이 같은 tf에 도착하도록)
      ax_[0] = compute_with_fixed_tf_1d(start_xyz.x(), goal_xyz.x(), tf_, acc_max, vel_max);
      ax_[1] = compute_with_fixed_tf_1d(start_xyz.y(), goal_xyz.y(), tf_, acc_max, vel_max);
      ax_[2] = compute_with_fixed_tf_1d(start_xyz.z(), goal_xyz.z(), tf_, acc_max, vel_max);

    } else {
      // mode 1: tf를 사용자가 고정
      tf_ = tf_fixed;
      ax_[0] = compute_with_fixed_tf_1d(start_xyz.x(), goal_xyz.x(), tf_, acc_max, vel_max);
      ax_[1] = compute_with_fixed_tf_1d(start_xyz.y(), goal_xyz.y(), tf_, acc_max, vel_max);
      ax_[2] = compute_with_fixed_tf_1d(start_xyz.z(), goal_xyz.z(), tf_, acc_max, vel_max);
    }
  }

  double duration() const { return tf_; }

  TrajectoryState3D compute(double t) const {
    TrajectoryState3D out;

    double x, xd, xdd;
    double y, yd, ydd;
    double z, zd, zdd;

    eval_axis(ax_[0], t, x, xd, xdd);
    eval_axis(ax_[1], t, y, yd, ydd);
    eval_axis(ax_[2], t, z, zd, zdd);

    out.position << x, y, z;
    out.velocity << xd, yd, zd;
    out.acceleration << xdd, ydd, zdd;

    return out;
  }

 private:
    int mode_{0};
    double tf_{0.0};
    LspbAxisParams ax_[3];
};

} // namespace ys_trajectory
