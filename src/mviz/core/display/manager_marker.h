
// Copyright 2023 MINIEYE
#ifndef MANAGER_MARKER_H__
#define MANAGER_MARKER_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "apa_state.pb.h"
#include "draw_box/box3d_vertex.h"
#include "freespace_obstacles.pb.h"
#include "parking_object.pb.h"
#include "parkingspace.pb.h"
#include "planning.pb.h"
#include "planningtohmi.pb.h"
class ManagerMarker {
 public:
  explicit ManagerMarker(const float& scale);
  ManagerMarker(const ManagerMarker&) = delete;
  ManagerMarker& operator=(const ManagerMarker&) = delete;
  ManagerMarker(ManagerMarker&&) = delete;
  ManagerMarker& operator=(ManagerMarker&&) = delete;
  ~ManagerMarker() noexcept;

  void InitMarker();
  void InitPncMarker();
  void InitUssParkingspaceMarker();
  bool UpdateGridMapMarker(const perception::FreespaceObstacles& grid_map);
  bool UpdateParkingSpaceMarker(const perception::ParkingSpace& parkingspace);
  bool UpdatePlanningMarker(const minieye::Planning& planning, float& body_width);
  bool UpdateSlotsSelectedAtt(const minieye::APAStateControl& apa_state_control);
  bool UpdateSlotsRecommendAtt(const minieye::PlanningToHMI& planning_to_hmi);
  bool UpdatePlanningState(float res, float cur_grer);
  bool UpdateUssParkingspaceMarker(const perception::ParkingSpace& uss_parkingspace);
  bool UpdateParkingTargetMarker(const minieye::parking::ObjectTrackList&);

  visualization_msgs::Marker grid_map_marker_;
  visualization_msgs::MarkerArray parkingspace_markers_;
  visualization_msgs::Marker parkingspace_marker_;

  visualization_msgs::Marker center_line_marker_;
  visualization_msgs::Marker parkingspace_marker_text_;
  visualization_msgs::Marker parkingspace_marker_stage_;

  visualization_msgs::Marker parkingspace_marker_rod_;
  visualization_msgs::Marker parkingspace_marker_rod_stage_;

  visualization_msgs::MarkerArray planning_markers_;
  visualization_msgs::Marker planning_marker_;

  visualization_msgs::Marker car_body_trace_marker_;
  visualization_msgs::Marker planning_state_marker_;
  visualization_msgs::Marker planning_bias_marker_;

  visualization_msgs::MarkerArray uss_parkingspace_markers_;
  visualization_msgs::Marker uss_parkingspace_marker_;
  visualization_msgs::Marker uss_parkingspace_marker_text_;
  visualization_msgs::Marker uss_parkingspace_center_line_marker_;

  visualization_msgs::MarkerArray parking_target_markers_;
  visualization_msgs::Marker parking_target_marker_;
  visualization_msgs::Marker parking_target_marker_text_;
  mviz_utils::Box3dVertex box3d_vertexs_;

  const float m_scale;
  float mod_lifetime_;

 public:
  void SetMarkerLifetime(const float& mod_lifetime);

 private:
  /* data */
};

#endif
