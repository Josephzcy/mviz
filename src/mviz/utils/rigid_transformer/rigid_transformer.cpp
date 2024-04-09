#include "rigid_transformer.h"

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "glog/logging.h"

namespace module {
namespace sensor_fusion {
namespace common {
// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
Eigen::Quaterniond GetQuaternion(const double roll, const double pitch, const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

}  // namespace common
}  // namespace sensor_fusion
}  // namespace module