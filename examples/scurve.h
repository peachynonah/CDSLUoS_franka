#pragma once

#include <Eigen/Dense>
// #include <eigen3/Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ys_trajectory_scurve {

// ---------------------------
// 결과 구조체 (3D 전용) + jerk
// ---------------------------
struct TrajectoryState3D_Jerk {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
};

// ---------------------------
// 내부: 축(1-DOF) S-curve(= jerk-limited) 파라미터
// - acceleration: trapezoid
// - jerk: piecewise constant
// ---------------------------
struct ScurveAxisParams {
  double x0 = 0.0;
  double x1 = 0.0;

  // limits (양수 크기)
  double jerk_max = 0.0;
  double acc_max  = 0.0;
  double vel_max  = 0.0;

  // 실제 사용값(양수 크기)
  double J = 0.0;  // <= jerk_max
  double A = 0.0;  // <= acc_max
  double V = 0.0;  // <= vel_max

  // segment times
  double tj = 0.0; // jerk segment time
  double ta = 0.0; // accel plateau time (a=+A)
  double tv = 0.0; // cruise time (a=0, v=V)
  double tf = 0.0; // total time = 4*tj + 2*ta + tv

  // direction sign
  double sgn = 1.0;
};

inline double sgn(double v) { return (v >= 0.0) ? 1.0 : -1.0; }

// 구간 jerk 상수일 때 적분(상태 업데이트)
// a(t)=a0+j t, v(t)=v0+a0 t + 1/2 j t^2, x(t)=x0+v0 t + 1/2 a0 t^2 + 1/6 j t^3
inline void integrate_const_jerk(double j, double dt, double& x, double& v, double& a) {
  x += v * dt + 0.5 * a * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;
  v += a * dt + 0.5 * j * dt * dt;
  a += j * dt;
}

// accel-half(가속 절반) 이동거리: s_acc = A*tj^2 + 1.5*A*tj*ta + 0.5*A*ta^2
inline double s_acc_half(double A, double tj, double ta) {
  return A * tj * tj + 1.5 * A * tj * ta + 0.5 * A * ta * ta;
}

// ---------------------------
// mode=0용: 주어진 (Jmax,Amax,Vmax)에서 1축 최소시간 S-curve 생성
// (start/end에서 v=a=0, 대칭 7-seg 가정)
// ---------------------------
inline ScurveAxisParams compute_min_time_scurve_1d(double x0, double x1,
                                                   double jerk_max,
                                                   double acc_max,
                                                   double vel_max) {
  if (jerk_max <= 0.0 || acc_max <= 0.0 || vel_max <= 0.0) {
    throw std::invalid_argument("jerk_max, acc_max, vel_max must be > 0");
  }

  ScurveAxisParams p;
  p.x0 = x0;
  p.x1 = x1;
  p.jerk_max = jerk_max;
  p.acc_max = acc_max;
  p.vel_max = vel_max;

  const double dx = x1 - x0;
  const double D  = std::abs(dx);
  p.sgn = sgn(dx);

  if (D == 0.0) {
    p.J = p.A = p.V = 0.0;
    p.tj = p.ta = p.tv = p.tf = 0.0;
    return p;
  }

  // V_switch = Amax^2 / Jmax
  // vel_max < V_switch이면 Amax를 못찍고(ta=0), A_peak = sqrt(V*J)
  const double V_switch = (acc_max * acc_max) / jerk_max;

  double tj = 0.0, ta = 0.0, A = 0.0, V = 0.0, tv = 0.0;

  if (vel_max < V_switch) {
    // ta=0, V = J*tj^2 => tj = sqrt(V/J)
    tj = std::sqrt(vel_max / jerk_max);
    ta = 0.0;
    A  = jerk_max * tj;      // A_peak
    V  = vel_max;            // v_peak
  } else {
    // A=Amax까지 올리고, 필요하면 plateau로 Vmax까지
    tj = acc_max / jerk_max;
    ta = vel_max / acc_max - tj; // >= 0 (수치상 음수면 0)
    if (ta < 0.0) ta = 0.0;
    A  = acc_max;
    V  = A * (ta + tj);
    if (V > vel_max) V = vel_max;
  }

  // 현재 (tj,ta,A)로 크루즈 없이 갈 때 필요한 거리
  const double D_need_no_cruise = 2.0 * s_acc_half(A, tj, ta);
  const double D_min_no_plateau = 2.0 * (A * tj * tj); // ta=0 기준

  if (D > D_need_no_cruise) {
    tv = (D - D_need_no_cruise) / V;
  } else {
    tv = 0.0;
    // 크루즈 없이 거리 맞추기: (가능하면 A,tj 유지하고 ta 조정)
    if (D >= D_min_no_plateau) {
      // D/A = 2*tj^2 + 3*tj*ta + ta^2  (ta>=0 해)
      // ta^2 + 3*tj*ta + (2*tj^2 - D/A)=0
      const double c = 2.0 * tj * tj - (D / A);
      const double disc = 9.0 * tj * tj - 4.0 * c;          // = tj^2 + 4D/A
      ta = 0.5 * (-3.0 * tj + std::sqrt(std::max(0.0, disc)));
      if (ta < 0.0) ta = 0.0;
      V = A * (ta + tj);
      if (V > vel_max) V = vel_max;
    } else {
      // 너무 짧아서 A도 못찍는 jerk-only(ta=0) : D = 2*J*tj^3
      tj = std::cbrt(D / (2.0 * jerk_max));
      ta = 0.0;
      A  = jerk_max * tj;
      V  = jerk_max * tj * tj;
    }
  }

  p.J = jerk_max;
  p.A = A;
  p.V = V;
  p.tj = tj;
  p.ta = ta;
  p.tv = tv;
  p.tf = 4.0 * tj + 2.0 * ta + tv;

  return p;
}

// ---------------------------
// 단일 축 S-curve: 시간 t에 대한 (pos, vel, acc, jerk) 계산
// - 7구간 jerk: +J,0,-J,0,-J,0,+J  (대칭)
// - 내부 계산은 양의 거리(0->D) 기준 후, 마지막에 부호/오프셋 적용
// ---------------------------
inline void eval_axis_scurve(const ScurveAxisParams& p, double t,
                            double& x, double& xd, double& xdd, double& xddd) {
  if (t < 0.0) t = 0.0;
  if (t > p.tf) t = p.tf;

  const double dx = p.x1 - p.x0;
  const double D  = std::abs(dx);

  if (p.tf == 0.0 || D == 0.0) {
    x = p.x1; xd = 0.0; xdd = 0.0; xddd = 0.0;
    return;
  }

  // relative motion: xr in [0,D]
  double xr = 0.0, vr = 0.0, ar = 0.0;
  double jr = 0.0;

  const double J = p.J;

  const double segT[7] = {p.tj, p.ta, p.tj, p.tv, p.tj, p.ta, p.tj};
  const double segJ[7] = {+J,   0.0, -J,   0.0, -J,   0.0, +J};

  double tt = t;
  for (int i = 0; i < 7; ++i) {
    const double dur = segT[i];
    if (dur <= 0.0) continue;

    if (tt > dur) {
      integrate_const_jerk(segJ[i], dur, xr, vr, ar);
      tt -= dur;
    } else {
      integrate_const_jerk(segJ[i], tt, xr, vr, ar);
      jr = segJ[i];
      tt = 0.0;
      break;
    }
  }

  if (t >= p.tf) {
    xr = D; vr = 0.0; ar = 0.0; jr = 0.0;
  }

  // 부호/오프셋 적용
  x    = p.x0 + p.sgn * xr;
  xd   = p.sgn * vr;
  xdd  = p.sgn * ar;
  xddd = p.sgn * jr;
}

// ---------------------------
// 3D S-curve Trajectory (x,y,z)
// mode=0: 최소시간 -> tf = max(tf_i) 동기화
// mode=1: tf_fixed 고정 (tf_fixed >= 최소 동기화 시간 필요)
// 구현은 "time scaling"으로 동기화/고정시간을 만족:
//   alpha_i = tf_i / tf_total (<=1)
//   x(t)=x_min(alpha t), v*=alpha, a*=alpha^2, j*=alpha^3
// ---------------------------
class ScurveAccTrapTrajectory3D {
 public:
  ScurveAccTrapTrajectory3D() = default;

  void init(const Eigen::Vector3d& start_xyz,
            const Eigen::Vector3d& goal_xyz,
            double jerk_max,
            double acc_max,
            double vel_max,
            int mode = 0,
            double tf_fixed = 0.0) {
    if (jerk_max <= 0.0 || acc_max <= 0.0 || vel_max <= 0.0) {
      throw std::invalid_argument("jerk_max, acc_max, vel_max must be > 0");
    }

    mode_ = mode;

    // 1) 축별 최소시간 파라미터
    ax_[0] = compute_min_time_scurve_1d(start_xyz.x(), goal_xyz.x(), jerk_max, acc_max, vel_max);
    ax_[1] = compute_min_time_scurve_1d(start_xyz.y(), goal_xyz.y(), jerk_max, acc_max, vel_max);
    ax_[2] = compute_min_time_scurve_1d(start_xyz.z(), goal_xyz.z(), jerk_max, acc_max, vel_max);

    const double Tmin_sync = std::max({ax_[0].tf, ax_[1].tf, ax_[2].tf});

    if (mode_ == 0) {
      tf_ = Tmin_sync;
    } else {
      if (tf_fixed <= 0.0) {
        throw std::invalid_argument("tf_fixed must be > 0 when mode=1");
      }
      if (tf_fixed + 1e-12 < Tmin_sync) {
        throw std::runtime_error(
            "Infeasible tf_fixed: tf_fixed < min required time under given limits.");
      }
      tf_ = tf_fixed;
    }
  }

  double duration() const { return tf_; }
  double min_sync_time() const { return std::max({ax_[0].tf, ax_[1].tf, ax_[2].tf}); }

  TrajectoryState3D_Jerk compute(double t) const {
    TrajectoryState3D_Jerk out;

    if (tf_ <= 0.0) {
      out.position << ax_[0].x1, ax_[1].x1, ax_[2].x1;
      out.velocity.setZero();
      out.acceleration.setZero();
      out.jerk.setZero();
      return out;
    }

    if (t < 0.0) t = 0.0;
    if (t > tf_) t = tf_;

    for (int i = 0; i < 3; ++i) {
      const double tfi = ax_[i].tf;

      if (tfi <= 0.0) {
        out.position(i) = ax_[i].x1;
        out.velocity(i) = 0.0;
        out.acceleration(i) = 0.0;
        out.jerk(i) = 0.0;
        continue;
      }

      const double alpha = tfi / tf_; // <=1
      double x, xd, xdd, xddd;
      eval_axis_scurve(ax_[i], alpha * t, x, xd, xdd, xddd);

      // chain rule scaling
      out.position(i)     = x;
      out.velocity(i)     = alpha * xd;
      out.acceleration(i) = (alpha * alpha) * xdd;
      out.jerk(i)         = (alpha * alpha * alpha) * xddd;
    }

    return out;
  }

 private:
  double time_duration_;
  Eigen::VectorXd start_pos_;
  Eigen::VectorXd end_pos_;
  Eigen::MatrixXd coefficients_; 
  int mode_{0};
  double tf_{0.0};
  ScurveAxisParams ax_[3];
};

} // namespace ys_trajectory
