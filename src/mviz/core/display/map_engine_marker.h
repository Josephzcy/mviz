#ifndef MAP_ENGINE_MARKER_H_
#define MAP_ENGINE_MARKER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "map_engine_response.pb.h"

//目标车位、planning、trajectory、end_pose
namespace havp::ap_slam {
class MapEngineMarker {
 public:
  explicit MapEngineMarker(const float& scale,const float& body_width);
  ~MapEngineMarker();
  MapEngineMarker(const MapEngineMarker&) = delete;
  MapEngineMarker& operator=(const MapEngineMarker&) = delete;
  MapEngineMarker(MapEngineMarker&&) = delete;
  MapEngineMarker& operator=(MapEngineMarker&&) = delete;

 public:
  bool Update(const Eigen::Isometry3d& enu_to_odom, const minieye::parking::MapEngineResponse& map_engine_response);
  void Init();

  const visualization_msgs::MarkerArray& GetMarkerArray() const;

 private:
  const float scale_;
  const float body_width_ {1.806};

 private:
  visualization_msgs::MarkerArray map_engine_markers_;
  visualization_msgs::Marker planning_marker_;
  visualization_msgs::Marker trajectory_marker_;

  visualization_msgs::Marker end_pose_marker_;
  visualization_msgs::Marker target_parkingspace_marker_;
};
}  // namespace havp::ap_slam

#endif