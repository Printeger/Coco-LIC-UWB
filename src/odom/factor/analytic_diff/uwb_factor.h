#pragma once

#include <ceres/ceres.h>
#include <odom/msg_manager.h>
#include <spline/trajectory.h>
#include <utils/parameter_struct.h>

#include "split_spline_view.h"

namespace cocolic {
namespace analytic_derivative {

class UWBFactorNURBS : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  UWBFactorNURBS(int64_t time_uwb_ns, const UwbData& uwb_data,
                 const std::pair<int, double>& su,
                 const Eigen::Matrix4d& blending_matrix,
                 const Eigen::Matrix4d& cumulative_blending_matrix,
                 const SO3d& S_GtoM, const Vec3d& p_GinM, const SO3d& S_UtoI,
                 const Vec3d& p_UinI, double weight)
      : time_uwb_ns_(time_uwb_ns),
        uwb_data_(uwb_data),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix),
        S_GtoM_(S_GtoM),
        p_GinM_(p_GinM),
        S_UtoI_(S_UtoI),
        p_UinI_(p_UinI),
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
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;
    // Extrinsic UWB to IMU
    Vec3d p_U = Vec3d::Zero();
    Vec3d p_IU = S_UtoI_ * p_U + p_UinI_;

    So3SplineView so3_spline_view;
    RdSplineView r3_spline_view;
    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
    // Calculate rotation matrix from IMU axis to global frame, and position
    if (jacobians) {
      S_ItoG = so3_spline_view.EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                               parameters, &J_R);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                            parameters + 4, &J_p);
    } else {
      S_ItoG = so3_spline_view.EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                               parameters, nullptr);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                            parameters + 4, nullptr);
    }
    Vec3d p_UinM = S_GtoM_ * (S_ItoG * p_U + p_IinG) + p_GinM_;
    // Calculate the position of UWB anchors in global frame
    for (auto anchor : uwb_data_.anchor_positions) {
      auto it = uwb_data_.anchor_distances.find(anchor.first);
      residuals[0] += it->second - (p_UinM - anchor.second).norm();
    }
    return true;
  }

 private:
  int64_t time_uwb_ns_;
  UwbData uwb_data_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
  Eigen::Vector3d uwb_measurement_;
  Eigen::Matrix3d K_;
  SO3d S_UtoI_;
  Vec3d p_UinI_;
  SO3d S_GtoM_;
  Vec3d p_GinM_;
  double weight_;
};

}  // namespace analytic_derivative
}  // namespace cocolic