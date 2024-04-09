#ifndef HAVP_PARKINGSPACE_POSTPROCESS_COMMON_POSE_INTERPLLATOR_H
#define HAVP_PARKINGSPACE_POSTPROCESS_COMMON_POSE_INTERPLLATOR_H

#include <queue>
#include <vector>

namespace module {
namespace post_process {
namespace common {

class PoseInterpolator {
 public:
  PoseInterpolator(const config::PoseInterpolatorConfig& pose_interpolator_config);

  PoseInterpolator(const PoseInterpolator& pose_interpolator) = delete;

  PoseInterpolator& operator=(const PoseInterpolator& pose_interpolator) = delete;

  void Process(const std::deque<TimedPose3D>& history_poses, const uint64_t& predict_tm, TimedPose3D& predicted_pose);

  void Slerp(const Rigid3d::Quaternion& pose1, const Rigid3d::Quaternion& pose2, Rigid3d::Quaternion& pose, float t);

 private:
  config::PoseInterpolatorConfig pose_interpolator_config_;
};
}  // namespace common
}  // namespace post_process
}  // namespace module
#endif