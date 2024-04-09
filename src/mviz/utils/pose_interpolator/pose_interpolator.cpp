#include "pose_interpolator.h"

#include <algorithm>

#include "glog/logging.h"
namespace module {
namespace sensor_fusion {
namespace common {
PoseInterpolator::PoseInterpolator(const config::PoseInterpolatorConfig& pose_interpolator_config)
    : pose_interpolator_config_(pose_interpolator_config) {}

void PoseInterpolator::Slerp(const Rigid3d::Quaternion& pose1, const Rigid3d::Quaternion& pose2,
                             Rigid3d::Quaternion& pose, float t) {
  float x1 = pose1.x();
  float y1 = pose1.y();
  float z1 = pose1.z();
  float w1 = pose1.w();
  float x2 = pose2.x();
  float y2 = pose2.y();
  float z2 = pose2.z();
  float w2 = pose2.w();

  float cosTheta = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2;
  if (cosTheta < 0) {
    x2 = -x2;
    y2 = -y2;
    z2 = -z2;
    w2 = -w2;
    cosTheta = -cosTheta;
  }

  float k1, k2;
  if (cosTheta > 0.9995) {
    k1 = 1.0 - t;
    k2 = t;
  } else {
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    float theta = atan2(sinTheta, cosTheta);
    k1 = sin((1.0 - t) * theta) / sinTheta;
    k2 = sin(t * theta) / sinTheta;
  }

  pose.x() = x1 * k1 + x2 * k2;
  pose.y() = y1 * k1 + y2 * k2;
  pose.z() = z1 * k1 + z2 * k2;
  pose.w() = w1 * k1 + w2 * k2;
}

void PoseInterpolator::Process(const std::deque<TimedPose3D>& history_poses, const uint64_t& predict_tm,
                               TimedPose3D& predicted_pose) {
  if (history_poses.empty()) {
    LOG(WARNING) << "history pose empty";
    return;
  }

  // < min
  // > max
  // size == 1
  // min <= x <= max
  if (predict_tm < history_poses.front().tm) {
    LOG_IF(ERROR, predict_tm < history_poses.front().tm)
        << "pred tm is preivous from history poses, pred time: " << predict_tm << ", "
        << "history earliest time: " << history_poses.front().tm;
    // diagnoser::CWrapperDiag::instance().report_error(CAPA_INPUT, FAPA_ODOMETRY_DATA_ERROR, EPPISIGHTINPUTABNORMAL);
    predicted_pose = history_poses.front();
  } else if (predict_tm > history_poses.back().tm) {
    LOG_IF(ERROR, predict_tm > history_poses.back().tm)
        << "pred tm is later from history poses, pred time: " << predict_tm << ", "
        << "history latest time: " << history_poses.back().tm;
    // diagnoser::CWrapperDiag::instance().report_error(CAPA_INPUT, FAPA_ODOMETRY_DATA_ERROR, EPPISIGHTINPUTABNORMAL);
    predicted_pose = history_poses.back();
  } else if (history_poses.size() <= 1) {
    predicted_pose = history_poses.back();
  } else {
    // front_tm <= pred_tm <= back_tm
    // 找upper bound，最小是history_poses.begin() + 1,最大是history_poses.end()
    auto it = std::upper_bound(
        history_poses.begin(), history_poses.end(), predict_tm,
        [](uint64_t target_time, const TimedPose3D& timed_pose) { return timed_pose.tm > target_time; });
    if (it == history_poses.end()) {
      it = history_poses.end() - 1;
    }

    const TimedPose3D& pose1 = *it;
    const TimedPose3D& pose0 = *(it - 1);
    double dt = pose1.tm - pose0.tm;
    double insert_dt = predict_tm - pose0.tm;
    LOG(INFO) << "insert_dt: " << insert_dt;
    double scale = insert_dt / dt;
    auto insert_double = [&scale](double x0, double x1, double& result) { result = x0 + scale * (x1 - x0); };
    Rigid3d::Vector translation;
    Rigid3d::Quaternion rotation;

    // 平移向量线性插值
    insert_double(pose0.pose.translation().x(), pose1.pose.translation().x(), translation.x());
    insert_double(pose0.pose.translation().y(), pose1.pose.translation().y(), translation.y());
    insert_double(pose0.pose.translation().z(), pose1.pose.translation().z(), translation.z());

    // 四元数球面插值
    Slerp(pose0.pose.rotation(), pose1.pose.rotation(), rotation, scale);

    predicted_pose.tm = predict_tm;
    predicted_pose.pose = Pose3D(translation, rotation);
  }
}

}  // namespace common
}  // namespace sensor_fusion
}  // namespace module