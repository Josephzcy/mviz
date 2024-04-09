#ifndef SENSOR_FUSION_COMMON_POSE_H
#define SENSOR_FUSION_COMMON_POSE_H

#include "odometry_3d.pb.h"
#include "rigid_transformer/rigid_transformer.h"

namespace module {
namespace sensor_fusion {
namespace common {
using Pose2D = Rigid2d;
using Pose2F = Rigid2f;
using Pose3D = Rigid3d;
using Pose3F = Rigid3f;

struct TimedPose2D {
  uint64_t tick;
  Pose2D pose;
};

struct TimedPose3D {
  uint64_t tick;
  Pose3D pose;
};

struct TimedPose2F {
  uint64_t tick;
  Pose2F pose;
};

struct TimedPose3F {
  uint64_t tick;
  Pose3F pose;
};

void TimedPose3DToOdom3D(const TimedPose3D& pose, minieye::Odometry3D* odom);

void Odom3DToTimedPose3D(const minieye::Odometry3D& odom, TimedPose3D* pose);

void Odom3DToPose3D(const minieye::Odometry3D& odom, Pose3D* pose);

}  // namespace common
}  // namespace sensor_fusion
}  // namespace module
#endif