#ifndef SENSOR_FUSION_COMMON_POINT_H
#define SENSOR_FUSION_COMMON_POINT_H

#include "glog/logging.h"
#include "rigid_transformer/rigid_transformer.h"
namespace module {
namespace sensor_fusion {
namespace common {
using Point2F = Rigid2f::Vector;
using Point2D = Rigid2d::Vector;
using Point3F = Rigid3f::Vector;
using Point3D = Rigid3d::Vector;

struct RodPoint {
  Point2D img_pt;
  Point3D vehicle_pt;
  Point3D odom_pt;
};

struct ParkingPoint {
  bool is_good;
  bool is_entry;
  Point2D img_pt;
  Point3D vehicle_pt;
  Point3D odom_pt;
};

template <typename FloatType>
double CrossProduct2D(const Eigen::Matrix<FloatType, 2, 1>& p0, const Eigen::Matrix<FloatType, 2, 1>& p1) {
  return p0.x() * p1.y() - p1.x() * p0.y();
}

// Projects 'point' onto the XY plane.
template <typename FloatType>
Eigen::Matrix<FloatType, 2, 1> PointProject2D(const Eigen::Matrix<FloatType, 3, 1>& point) {
  return point.template head<2>();
}

template <typename FloatType, int Rows>
double GetRadianBetweenVectors(const Eigen::Matrix<FloatType, Rows, 1>& v1,
                               const Eigen::Matrix<FloatType, Rows, 1>& v2) {
  LOG_IF(ERROR, v1.norm() < 1e-6 || v2.norm() < 1e-6) << "norm tool small, v1 " << v1.norm() << ", v2: " << v2.norm();
  return std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

}  // namespace common
}  // namespace sensor_fusion
}  // namespace module
#endif