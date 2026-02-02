#pragma once

#include <Eigen/Dense>

/**
 * @brief Placeholder for Sigmoid-CLBF safe acceleration term.
 *
 * This project version intentionally removes all CLBF/asafe logic.
 * The function keeps the original signature so existing example code
 * can compile unchanged, but always returns 0.0.
 *
 * If you later want to re-enable the CLBF, implement the actual math
 * in sigmoid_clbf.cpp.
 */
double getasafe(
    const Eigen::Matrix<double, 2, 2>& Q_matrix,
    const Eigen::Matrix<double, 2, 2>& P_matrix,
    const Eigen::Matrix<double, 2, 1>& state_error,
    double clbf_slope_l_,
    double unsafe_d_,
    double clbf_margin_delta_,
    double clbf_weight_theta_,
    double cart_pos_current);