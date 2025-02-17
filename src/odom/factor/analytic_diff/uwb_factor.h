#pragma once

#include <ceres/ceres.h>
#include <odom/msg_manager.h>
#include <spline/trajectory.h>
#include <utils/parameter_struct.h>

namespace cocolic {
namespace analytic_derivative {

class UWBFactorNURBS : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UWBFactorNURBS(int64_t time_ns, const std::pair<int, double>& segment,
                 const Eigen::Matrix4d& blending_matrix,
                 const Eigen::Matrix4d& cumulative_blending_matrix,
                 const UwbData& uwb_measurement, double weight)
      : time_ns_(time_ns),
        segment_(segment),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix),
        uwb_measurement_(uwb_measurement),
        weight_(weight) {
    set_num_residuals(1);

    size_t kont_num = 4;
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    // TODO: Implement UWB factor evaluation
    // 1. Get control points from parameters
    // 2. TODO: Calculate predicted UWB measurement using B-spline!!!
    // 3. Compute residual between predicted and actual UWB measurement
    // 4. If jacobians requested, compute derivatives w.r.t control points

    // Example structure:
    Eigen::Map<const SO3d> R_ItoG(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> p_IinG(parameters[1]);

    // Predicted UWB measurement based on current pose
    Eigen::Vector3d predicted_uwb = p_IinG;  // TODO: Add UWB measurement model

    // Compute residual
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = weight_ * (predicted_uwb - uwb_measurement_);

    // TODO: Compute jacobians if requested
    if (jacobians) {
      if (jacobians[0]) {
        // Derivative w.r.t rotation
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> j_rot(
            jacobians[0]);
        // TODO: Implement rotation jacobian
      }
      if (jacobians[1]) {
        // Derivative w.r.t position
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> j_pos(
            jacobians[1]);
        // TODO: Implement position jacobian
      }
    }

    return true;
  }

 private:
  int64_t time_ns_;
  UwbData uwb_data_;
  std::pair<int, double> segment_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
  Eigen::Vector3d uwb_measurement_;
  Eigen::Matrix3d K_;
  double weight_;
};

}  // namespace analytic_derivative
}  // namespace cocolic