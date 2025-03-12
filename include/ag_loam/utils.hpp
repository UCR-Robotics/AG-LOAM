// The AG-LOAM project
// Hanzhe Teng, April 2022

#pragma once

#include <chrono>
#include <iostream>
#include <string>

#include <Eigen/Core>

namespace ag_loam {

inline double GetAngleRadian(const Eigen::Matrix4d& pose) {
  return fabs(acos(0.5 * (pose.coeff(0, 0) + pose.coeff(1, 1) + pose.coeff(2, 2) - 1)));
}

inline double GetAngleDegree(const Eigen::Matrix4d& pose) {
  return fabs(acos(0.5 * (pose.coeff(0, 0) + pose.coeff(1, 1) + pose.coeff(2, 2) - 1))) * 180.0 / M_PI;
}

inline double GetTranslation(const Eigen::Matrix4d& pose) { return pose.block<3, 1>(0, 3).norm(); }

inline Eigen::Matrix4d GetInverseTransform(const Eigen::Matrix4d& transform) {
  Eigen::Matrix4d inverse = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d rotation(transform.block<3, 3>(0, 0));
  Eigen::Vector3d translation(transform.block<3, 1>(0, 3));
  inverse.block<3, 3>(0, 0) = rotation.transpose();
  inverse.block<3, 1>(0, 3) = -rotation.transpose() * translation;
  return inverse;
}

template <typename PointT>
inline double DistanceAlongNormal(const PointT& a, const PointT& b) {
  Eigen::Vector3d va(a.x, a.y, a.z);
  Eigen::Vector3d vb(b.x, b.y, b.z);
  return std::abs((va - vb).dot(Eigen::Vector3d(a.normal_x, a.normal_y, a.normal_z)));
}

}  // namespace ag_loam
