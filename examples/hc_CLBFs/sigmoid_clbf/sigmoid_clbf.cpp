#include "sigmoid_clbf/sigmoid_clbf.h"

#include <cmath>

double getasafe(
    const Eigen::Matrix<double, 2, 2>& Q_matrix,
    const Eigen::Matrix<double, 2, 2>& P_matrix,
    const Eigen::Matrix<double, 2, 1>& state_error,
    double clbf_slope_l,
    double unsafe_d,
    double clbf_margin_delta,
    double clbf_weight_theta,
    double cart_pos_current) {

  // B matrix (second-order system)
  Eigen::Matrix<double, 2, 1> B_matrix;
  B_matrix << 0.0, 1.0;

  // CLF
  const double CLF_V =
      0.5 * (state_error.transpose() * P_matrix * state_error)(0, 0);

  const double temp_Q =
      (state_error.transpose() * Q_matrix * state_error)(0, 0);

  // Sigmoid activation
  const double sigma =
      1.0 / (1.0 + std::exp(clbf_slope_l *
              (cart_pos_current - unsafe_d - 0.5 * clbf_margin_delta)));

  const double nonlinear_weight =
      1.0 + clbf_weight_theta * sigma;

  // CLBF
  const double CLBF_W = nonlinear_weight * CLF_V;

  // Lie derivatives (scalar form)
  const double alpha =
      -nonlinear_weight * temp_Q
      -clbf_weight_theta * CLF_V *
        (clbf_slope_l * sigma * (1.0 - sigma) * state_error(1));

  const double beta =
      nonlinear_weight *
      (state_error.transpose() * P_matrix * B_matrix)(0, 0);

  const double beta_sq  = beta * beta;
  const double beta_4th = beta_sq * beta_sq;

  // Safe acceleration
  const double asafe =
      -((alpha + std::sqrt(alpha * alpha + beta_4th)) / beta_sq) * beta;

  return asafe;
}