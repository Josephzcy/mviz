#include "map_engine_marker.h"

namespace havp::ap_slam {

MapEngineMarker::MapEngineMarker(const float& scale, const float& body_width) : scale_(scale), body_width_(body_width) {
  Init();
}
const visualization_msgs::MarkerArray& MapEngineMarker::GetMarkerArray() const { return map_engine_markers_; }

bool MapEngineMarker::Update(const Eigen::Isometry3d& enu_to_odom,
                             const minieye::parking::MapEngineResponse& map_engine_response) {
  std::uint16_t marker_size = map_engine_markers_.markers.size();
  for (auto index = 0; index < marker_size; index++) {
    map_engine_markers_.markers[index].points.clear();
  }
  map_engine_markers_.markers.clear();

  // 2. trajectory_marker_
  trajectory_marker_.points.clear();
  trajectory_marker_.id = 0;
  if (map_engine_response.trajectory().size() >= 2) {
    for (auto point : map_engine_response.trajectory()) {
      geometry_msgs::Point pt;
      pt.x = point.x();
      pt.y = point.y();
      pt.z = point.z();
      Eigen::Vector3d original_vector(pt.x, pt.y, pt.z);
      Eigen::Vector3d rotation_vector = enu_to_odom * original_vector;
      pt.x = rotation_vector.x();
      pt.y = rotation_vector.y();
      pt.z = rotation_vector.z();
      trajectory_marker_.points.push_back(pt);
    }
    trajectory_marker_.scale.x = body_width_;
    map_engine_markers_.markers.push_back(trajectory_marker_);
  } else {
    ROS_INFO_STREAM("map_engine_response trajectory size less than 2");
  }

  auto parkslot = map_engine_response.target_parking_slot();
  if (parkslot.id() > 0) {
    for (const auto& corner_pt : parkslot.corner_pts()) {
      geometry_msgs::Point pt;
      pt.x = corner_pt.odom_pt().x();
      pt.y = corner_pt.odom_pt().y();

      Eigen::Vector3d original_vector(pt.x, pt.y, pt.z);
      Eigen::Vector3d rotation_vector = enu_to_odom * original_vector;
      pt.x = rotation_vector.x();
      pt.y = rotation_vector.y();
      pt.z = rotation_vector.z();
      target_parkingspace_marker_.points.push_back(pt);
      target_parkingspace_marker_.id = parkslot.id();
    }
    target_parkingspace_marker_.points.push_back(target_parkingspace_marker_.points.front());
  } else {
    ROS_INFO_STREAM("do not find target parking slot");
  }

  // 4. planning 不用坐标系转换(odome)
  geometry_msgs::Point pt;
  auto planning_trajectory = map_engine_response.planning().planning_trajectory();
  planning_marker_.id = 0;  // 0表示清除历史轨迹
  planning_marker_.points.clear();
  for (const auto& trajectory : planning_trajectory.trajectory()) {
    pt.x = trajectory.path_point().x();
    pt.y = trajectory.path_point().y();
    pt.z = trajectory.path_point().z();

    planning_marker_.points.push_back(pt);
  }
  map_engine_markers_.markers.push_back(planning_marker_);

  std::cout << "===update end_points" << std::endl;
  double roll = map_engine_response.mapping_end_pose().roll();
  double pitch = map_engine_response.mapping_end_pose().pitch();
  double yaw = map_engine_response.mapping_end_pose().yaw();

  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  // Todo::这里只需要显示，不需要变换，固不需要单位化
  geometry_msgs::Pose original_pose;
  original_pose.position.x = map_engine_response.mapping_end_pose().x();
  original_pose.position.y = map_engine_response.mapping_end_pose().y();
  original_pose.position.z = map_engine_response.mapping_end_pose().z();
  original_pose.orientation.w = q.w();
  original_pose.orientation.x = q.x();
  original_pose.orientation.y = q.y();
  original_pose.orientation.z = q.z();

  // 转到odom 坐标系下
  Eigen::Isometry3d original_isometry = Eigen::Isometry3d::Identity();
  original_isometry.translation() =
      Eigen::Vector3d(original_pose.position.x, original_pose.position.y, original_pose.position.z);
  original_isometry.linear() = q.toRotationMatrix();

  // 应用平移变换到原始姿态
  Eigen::Isometry3d transformed_isometry = enu_to_odom * original_isometry;

  // 将变换后的姿态转换回geometry_msgs::Pose对象
  geometry_msgs::Pose transformed_pose;
  transformed_pose.position.x = transformed_isometry.translation().x();
  transformed_pose.position.y = transformed_isometry.translation().y();
  transformed_pose.position.z = transformed_isometry.translation().z();
  Eigen::Quaterniond q_transformed(transformed_isometry.linear());
  transformed_pose.orientation.x = q_transformed.x();
  transformed_pose.orientation.y = q_transformed.y();
  transformed_pose.orientation.z = q_transformed.z();
  transformed_pose.orientation.w = q_transformed.w();

  std::cout << "transformed_pose.x:" << transformed_pose.position.x << std::endl;
  std::cout << "transformed_pose.y:" << transformed_pose.position.y << std::endl;
  std::cout << "transformed_pose.z:" << transformed_pose.position.z << std::endl;
  std::cout << "transformed_pose.orientation.x:" << transformed_pose.orientation.x << std::endl;
  std::cout << "transformed_pose.orientation.y:" << transformed_pose.orientation.y << std::endl;
  std::cout << "transformed_pose.orientation.z:" << transformed_pose.orientation.z << std::endl;
  std::cout << "transformed_pose.orientation.w:" << transformed_pose.orientation.w << std::endl;

  end_pose_marker_.pose = transformed_pose;
  // end_pose_marker_.id++;

  map_engine_markers_.markers.push_back(end_pose_marker_);

  return true;
}

void MapEngineMarker::Init() {
  std::uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker_.header.frame_id = "odome";
  trajectory_marker_.header.stamp = ros::Time::now();
  trajectory_marker_.ns = "map_engine_trajectory";
  trajectory_marker_.type = shape;
  trajectory_marker_.action = visualization_msgs::Marker::ADD;
  trajectory_marker_.lifetime = ros::Duration();
  trajectory_marker_.pose.orientation.w = 1;
  trajectory_marker_.scale.x = scale_ * 4;
  trajectory_marker_.scale.y = scale_ * 4;
  trajectory_marker_.color.r = 1.0f;
  trajectory_marker_.color.g = 1.0f;
  trajectory_marker_.color.b = 1.0f;
  trajectory_marker_.color.a = 0.4f;  // 轨迹0.4

  planning_marker_ = trajectory_marker_;
  planning_marker_.ns = "map_engine_planning";
  planning_marker_.color.r = 0.75f;
  planning_marker_.color.g = 0.5f;
  planning_marker_.color.b = 0.0f;
  planning_marker_.color.a = 0.7f;

  target_parkingspace_marker_ = planning_marker_;
  target_parkingspace_marker_.ns = "map_engine_target_parkingspace";

  shape = visualization_msgs::Marker::CUBE;
  end_pose_marker_ = planning_marker_;
  end_pose_marker_.ns = "map_engine_end_pose";
  end_pose_marker_.type = shape;
  end_pose_marker_.color.r = 1.0f;
  end_pose_marker_.color.g = 0.0f;
  end_pose_marker_.color.b = 0.0f;
  end_pose_marker_.color.a = 1.0f;

  end_pose_marker_.scale.x = body_width_;
  end_pose_marker_.scale.y = body_width_;
  end_pose_marker_.scale.z = body_width_;
}

MapEngineMarker::~MapEngineMarker() {
  planning_marker_.points.clear();
  trajectory_marker_.points.clear();
  end_pose_marker_.points.clear();
  target_parkingspace_marker_.points.clear();
  map_engine_markers_.markers.clear();
}

}  // namespace havp::ap_slam