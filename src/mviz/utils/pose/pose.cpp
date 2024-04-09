#include "pose.h"

namespace module {
namespace sensor_fusion {
namespace common {

void TimedPose3DToOdom3D(const TimedPose3D& pose, minieye::Odometry3D* odom) {
  // tick
  odom->set_tick(pose.tick);
  // data
  minieye::Quaternion* quat = odom->mutable_orientation();
  quat->set_qw(pose.pose.rotation().w());
  quat->set_qx(pose.pose.rotation().x());
  quat->set_qy(pose.pose.rotation().y());
  quat->set_qz(pose.pose.rotation().z());
  common::Rigid3<double>::Vector euler_angle = common::GetRollPitchYaw(pose.pose.rotation());
  // 将 roll、 pitch置 0
  minieye::Pose6d* pose6d = odom->mutable_pose();
  pose6d->set_x(pose.pose.translation().x());
  pose6d->set_y(pose.pose.translation().y());
  pose6d->set_z(pose.pose.translation().z());
  pose6d->set_roll(euler_angle(0));
  pose6d->set_pitch(euler_angle(1));
  pose6d->set_yaw(euler_angle(2));
}

void Odom3DToTimedPose3D(const minieye::Odometry3D& odom, TimedPose3D* pose) {
  // tick
  pose->tick = odom.tick();
  // data
  const minieye::Quaternion& quat = odom.orientation();
  const minieye::Pose6d& pose6d = odom.pose();
  Eigen::Quaternion<double> rotation(quat.qw(), quat.qx(), quat.qy(), quat.qz());
  Eigen::Vector3d translation(pose6d.x(), pose6d.y(), pose6d.z());
  pose->pose = common::Pose3D(translation, rotation);
}

void Odom3DToPose3D(const minieye::Odometry3D& odom, Pose3D* pose) {
  const minieye::Quaternion& quat = odom.orientation();
  const minieye::Pose6d& pose6d = odom.pose();
  Eigen::Quaternion<double> rotation(quat.qw(), quat.qx(), quat.qy(), quat.qz());
  Eigen::Vector3d translation(pose6d.x(), pose6d.y(), pose6d.z());
  *pose = common::Pose3D(translation, rotation);
}

}  // namespace common
}  // namespace sensor_fusion
}  // namespace module