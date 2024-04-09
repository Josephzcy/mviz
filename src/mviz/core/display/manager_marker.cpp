#include "display/manager_marker.h"

#include <geometry_msgs/Point.h>

#include "category_color_mapping.h"
#include "viz_struct.hpp"

ManagerMarker::ManagerMarker(const float& scale) : m_scale(scale) {
  InitMarker();
  InitPncMarker();
  InitUssParkingspaceMarker();
}

ManagerMarker::~ManagerMarker() {
  grid_map_marker_.points.clear();
  parkingspace_markers_.markers.clear();
  planning_markers_.markers.clear();
  car_body_trace_marker_.points.clear();
  planning_state_marker_.points.clear();
}

void ManagerMarker::SetMarkerLifetime(const float& mod_lifetime) { mod_lifetime_ = mod_lifetime; }

void ManagerMarker::InitMarker() {
  // todo:gripmap show time is 0.08s,scale is 0.2f
  uint32_t shape = visualization_msgs::Marker::POINTS;
  grid_map_marker_.header.frame_id = "odome";
  grid_map_marker_.header.stamp = ros::Time::now();
  grid_map_marker_.ns = "grid_map";
  grid_map_marker_.type = shape;
  grid_map_marker_.action = visualization_msgs::Marker::ADD;
  grid_map_marker_.lifetime = ros::Duration();
  grid_map_marker_.pose.orientation.w = 1;
  grid_map_marker_.scale.x = m_scale * 4;
  grid_map_marker_.scale.y = m_scale * 4;
  grid_map_marker_.color.r = 0.0f;
  grid_map_marker_.color.g = 1.0f;
  grid_map_marker_.color.b = 1.0f;
  grid_map_marker_.color.a = 1.0f;

  // todo:show parking slots with rod and center line
  shape = visualization_msgs::Marker::LINE_STRIP;

  parkingspace_marker_ = grid_map_marker_;
  parkingspace_marker_.ns = "parkingspace";
  parkingspace_marker_.id = 0;
  parkingspace_marker_.type = shape;

  parkingspace_marker_.lifetime = ros::Duration();

  parkingspace_marker_.scale.x = m_scale;  // todo::扫描到的颜色为光绿色，选中为绿色
  parkingspace_marker_.color.r = 0.56f;
  parkingspace_marker_.color.g = 0.93f;
  parkingspace_marker_.color.b = 0.56f;

  // todo::rod
  shape = visualization_msgs::Marker::LINE_STRIP;
  parkingspace_marker_rod_ = parkingspace_marker_;
  parkingspace_marker_rod_.points.clear();

  parkingspace_marker_rod_.type = shape;
  parkingspace_marker_rod_.ns = "parkingspace_rod";
  parkingspace_marker_rod_.id = 0;

  // todo:center line (white)
  center_line_marker_ = parkingspace_marker_rod_;
  center_line_marker_.points.clear();
  center_line_marker_.ns = "parkingspace_center_line";

  center_line_marker_.id = 0;
  center_line_marker_.color.r = 1.0f;
  center_line_marker_.color.g = 0.8f;
  center_line_marker_.color.b = 0.8f;
  center_line_marker_.color.a = 0.8f;

  // todo:parkingSpace id text,text 需要更新实时更新位置和车位的四个角点
  shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
  parkingspace_marker_text_ = parkingspace_marker_;
  parkingspace_marker_text_.ns = "parkingspace_text";
  parkingspace_marker_text_.type = shape;
  parkingspace_marker_text_.id = 0;
  parkingspace_marker_text_.lifetime = ros::Duration(0);

  parkingspace_marker_text_.scale.z = 0.3f;
  parkingspace_marker_text_.color.r = 1.0f;
  parkingspace_marker_text_.color.g = 1.0f;
  parkingspace_marker_text_.color.b = 0.0f;

  parkingspace_marker_stage_ = parkingspace_marker_text_;
  parkingspace_marker_stage_.ns = "parkingspace_stage";

  parkingspace_marker_rod_stage_ = parkingspace_marker_text_;
  parkingspace_marker_rod_stage_.ns = "parkingspace_rod_stage";

  parkingspace_markers_.markers.push_back(parkingspace_marker_);
  parkingspace_markers_.markers.push_back(center_line_marker_);
  parkingspace_markers_.markers.push_back(parkingspace_marker_text_);
  parkingspace_markers_.markers.push_back(parkingspace_marker_stage_);

  parkingspace_markers_.markers.push_back(parkingspace_marker_rod_);
  parkingspace_markers_.markers.push_back(parkingspace_marker_rod_stage_);

  // parking_target
  parking_target_marker_ = grid_map_marker_;
  parking_target_marker_.type = visualization_msgs::Marker::LINE_LIST;
  parking_target_marker_.ns = "parking_target";
  parking_target_marker_.scale.x = 0.1;
  parking_target_marker_.scale.y = 0.0;
  parking_target_marker_.scale.z = 0.0;
  parking_target_marker_.color.b = ORANGE[0] / 255.0;
  parking_target_marker_.color.g = ORANGE[1] / 255.0;
  parking_target_marker_.color.r = ORANGE[2] / 255.0;
  parking_target_marker_.color.a = 0.8;

  parking_target_marker_text_ = parking_target_marker_;
  parking_target_marker_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  parking_target_marker_text_.ns = "parking_target_text";
  parking_target_marker_text_.scale.x = 0.5;
  parking_target_marker_text_.scale.y = 0.5;
  parking_target_marker_text_.scale.z = 0.5;
}

bool ManagerMarker::UpdateGridMapMarker(const perception::FreespaceObstacles& grid_map) {
  // todo:delete last grid_map and push cur time grid_map
  std::uint32_t grid_map_size = grid_map.obstacles_size();
  if (grid_map_size == 0) {
    ROS_WARN_STREAM("cur grid_map is empty");
    return false;
  }

  grid_map_marker_.points.clear();
  grid_map_marker_.colors.clear();
  auto color_map = CategoryColorMapping::GetColorMap();

  for (auto obstacle_index = 0; obstacle_index < grid_map.obstacles_size(); obstacle_index++) {
    auto obstacle = grid_map.obstacles(obstacle_index);
    grid_map_marker_.id = 0;

    for (auto point_index = 0; point_index < obstacle.points_size(); point_index++) {
      geometry_msgs::Point pt;
      pt.x = obstacle.points(point_index).x();
      pt.y = obstacle.points(point_index).y();
      if (std::isnan(pt.x) || std::isnan(pt.y)) {
        ROS_ERROR_STREAM("current gridmap exists nan points");
        return false;
      }
      grid_map_marker_.points.push_back(pt);
      if (obstacle.class_() == perception::kVision) {
        std::uint8_t vision_class = obstacle.category();
        // std::cout << "vision_class:" << static_cast<int>(vision_class) << std::endl;
        grid_map_marker_.colors.push_back(
            color_map.at(static_cast<CategoryColorMapping::MarkerColor>(vision_class + 1)));  //
      } else if (obstacle.class_() == perception::kUss) {
        std::uint8_t uss_class = obstacle.uss_class() > 12 ? 0 : color_map.size() - 1;
        // std::cout << "uss_class:" << static_cast<int>(uss_class) << std::endl;

        grid_map_marker_.colors.push_back(color_map.at(static_cast<CategoryColorMapping::MarkerColor>(uss_class)));
      }
    }
  }
  // todo:grid_map line is close
  if (grid_map_marker_.points.size() == 0) {
    return false;
  }

  grid_map_marker_.header.stamp = ros::Time::now();
  return true;
}

bool ManagerMarker::UpdateParkingSpaceMarker(const perception::ParkingSpace& parkingspace) {
  std::uint16_t marker_size = parkingspace_markers_.markers.size();
  std::uint32_t parkingspace_size = parkingspace.parkslots_size();

  if (parkingspace_size == 0) {
    ROS_WARN_STREAM("cur parkingSpace is empty");
    return false;
  }

  for (auto index = 0; index < marker_size; index++) {
    parkingspace_markers_.markers[index].points.clear();
  }
  parkingspace_markers_.markers.clear();

  // todo:update slots,every correspending to a marker
  parkingspace_marker_.header.stamp = parkingspace_marker_text_.header.stamp = ros::Time::now();
  parkingspace_marker_rod_.header.stamp = ros::Time::now();

  static std::uint32_t corner_id = 1;
  for (auto park_slot_index = 0; park_slot_index < parkingspace.parkslots_size(); park_slot_index++) {
    const perception::ParkingSlot& parking_slot = parkingspace.parkslots(park_slot_index);
    auto corner_point_size = parking_slot.corner_pts_size();
    if (corner_point_size < 4) {
      continue;
    }
    // todo:update until next slots
    parkingspace_marker_.points.clear();
    center_line_marker_.points.clear();
    parkingspace_marker_rod_.points.clear();

    std::uint32_t slot_id = parking_slot.id();
    parkingspace_marker_.id = slot_id;
    parkingspace_marker_text_.id = slot_id;
    center_line_marker_.id = slot_id;
    parkingspace_marker_rod_.id = slot_id;
    parkingspace_marker_rod_stage_.id = slot_id;

    if (parking_slot.occupied()) {
      parkingspace_marker_rod_.color.r = parkingspace_marker_.color.r = 1.0f;
      parkingspace_marker_rod_.color.g = parkingspace_marker_.color.g = 0.0f;
      parkingspace_marker_text_.color.r = 1.0f;
      parkingspace_marker_text_.color.g = 0.0f;
    } else {
      parkingspace_marker_rod_.color.r = 0.0f;  // todo:默认颜色
      parkingspace_marker_rod_.color.g = 1.0f;
      parkingspace_marker_.color.r = 0.81f;  // todo:灰颜色
      parkingspace_marker_.color.g = 0.81f;
      parkingspace_marker_.color.b = 0.81f;
      parkingspace_marker_text_.color.r = 1.0f;
      parkingspace_marker_text_.color.g = 1.0f;
    }

    visualization_msgs::Marker corner_id_marker = parkingspace_marker_text_;
    corner_id_marker.ns = "corner_text";

    for (auto corner_point_index = 0; corner_point_index < corner_point_size; corner_point_index++) {
      geometry_msgs::Point pt;
      pt.x = parking_slot.corner_pts(corner_point_index).odom_pt().x();
      pt.y = parking_slot.corner_pts(corner_point_index).odom_pt().y();
      if (std::isnan(pt.x) || std::isnan(pt.y)) {
        return false;
      }
      parkingspace_marker_.points.push_back(pt);

      // todo:: show corner point id

      corner_id_marker.id = corner_id++;
      corner_id_marker.pose.position.x = pt.x;
      corner_id_marker.pose.position.y = pt.y;
      corner_id_marker.text = std::to_string(parking_slot.corner_pts(corner_point_index).id());
      parkingspace_markers_.markers.push_back(corner_id_marker);
    }
    parkingspace_marker_.points.push_back(parkingspace_marker_.points.front());

    // todo: center line  for every slots
    geometry_msgs::Point centerPoint;
    centerPoint.x = (parkingspace_marker_.points[0].x + parkingspace_marker_.points[3].x) * 0.5;
    centerPoint.y = (parkingspace_marker_.points[0].y + parkingspace_marker_.points[3].y) * 0.5;
    centerPoint.z = (parkingspace_marker_.points[0].z + parkingspace_marker_.points[3].z) * 0.5;
    center_line_marker_.points.push_back(centerPoint);

    centerPoint.x = (parkingspace_marker_.points[1].x + parkingspace_marker_.points[2].x) * 0.5;
    centerPoint.y = (parkingspace_marker_.points[1].y + parkingspace_marker_.points[2].y) * 0.5;
    centerPoint.z = (parkingspace_marker_.points[1].z + parkingspace_marker_.points[2].z) * 0.5;
    center_line_marker_.points.push_back(centerPoint);

    // todo: slots rot corodinate for every slots
    if (parking_slot.exist_rod()) {
      std::uint32_t rod_size = parking_slot.rod_pts_size();

      if (rod_size == 2) {
        for (size_t rod_index = 0; rod_index < rod_size; rod_index++) {
          geometry_msgs::Point pt;
          pt.x = parking_slot.rod_pts(rod_index).odom_pt().x();
          pt.y = parking_slot.rod_pts(rod_index).odom_pt().y();
          parkingspace_marker_rod_.points.push_back(pt);
        }
        // stage限位杆增加stage
        parkingspace_marker_rod_stage_.pose.position.x =
            (parkingspace_marker_rod_.points[0].x + parkingspace_marker_rod_.points[1].x) * 0.5;
        parkingspace_marker_rod_stage_.pose.position.y =
            (parkingspace_marker_rod_.points[0].y + parkingspace_marker_rod_.points[1].y) * 0.5;
        parkingspace_marker_rod_stage_.text = "rs:" + std::to_string(parking_slot.rod_stage());
        parkingspace_markers_.markers.push_back(parkingspace_marker_rod_);
        parkingspace_markers_.markers.push_back(parkingspace_marker_rod_stage_);
      } else {
        ROS_ERROR_STREAM("Rod size is error");
      }
    }

    // todo: slots id corodinate for every slots
    parkingspace_marker_text_.pose.position.x =
        (parkingspace_marker_.points[0].x + parkingspace_marker_.points[2].x) * 0.5;
    parkingspace_marker_text_.pose.position.y =
        (parkingspace_marker_.points[0].y + parkingspace_marker_.points[2].y) * 0.5;

    // todo::coordinate of fixed two ratio dividing point

    geometry_msgs::Point point_first;
    point_first.x = (parkingspace_marker_.points[0].x + parkingspace_marker_.points[3].x) * 0.5;
    point_first.y = (parkingspace_marker_.points[0].y + parkingspace_marker_.points[3].y) * 0.5;

    geometry_msgs::Point point_second;
    point_second.x = (parkingspace_marker_.points[1].x + parkingspace_marker_.points[2].x) * 0.5;
    point_second.y = (parkingspace_marker_.points[1].y + parkingspace_marker_.points[2].y) * 0.5;

    parkingspace_marker_stage_.pose.position.x = (point_first.x + 2 * point_second.x) / 3;
    parkingspace_marker_stage_.pose.position.y = (point_first.y + 2 * point_second.y) / 3;

    std::string ps_type_text = "";
    switch (parking_slot.type()) {
      case 1:
        ps_type_text = "V";
        break;
      case 2:
        ps_type_text = "H";
        break;

      case 3:
        ps_type_text = "K";
        break;
      default:
        ps_type_text = "U";
        break;
    }

    parkingspace_marker_text_.text = std::to_string(parkingspace_marker_.id) + ps_type_text;
    parkingspace_marker_stage_.text =
        "ps:" + std::to_string(parking_slot.parking_stage()) + ",ss:" + std::to_string(parking_slot.search_stage());

    ROS_ERROR_STREAM("parkingspace_marker_stage_.text:" << parkingspace_marker_stage_.text);
    parkingspace_markers_.markers.push_back(parkingspace_marker_);
    parkingspace_markers_.markers.push_back(parkingspace_marker_text_);
    parkingspace_markers_.markers.push_back(parkingspace_marker_stage_);

    parkingspace_markers_.markers.push_back(center_line_marker_);

    // todo: update Rod posititon for every slots
  }
  return true;
}

void ManagerMarker::InitPncMarker() {
  std::uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
  planning_marker_.header.frame_id = "odome";  // 车身坐标系
  planning_marker_.header.stamp = ros::Time::now();

  planning_marker_.ns = "planning";
  planning_marker_.type = shape;

  planning_marker_.action = visualization_msgs::Marker::ADD;
  planning_marker_.lifetime = ros::Duration();
  planning_marker_.scale.x = m_scale;
  planning_marker_.color.r = 1.0f;
  planning_marker_.color.g = 0.0f;
  planning_marker_.color.b = 0.0f;
  planning_marker_.color.a = 1.0f;

  planning_marker_.pose.orientation.w = 1;

  car_body_trace_marker_ = planning_marker_;
  car_body_trace_marker_.ns = "car_body_trace";
  car_body_trace_marker_.color.r = 0.32f;
  car_body_trace_marker_.color.g = 1.0f;
  car_body_trace_marker_.color.b = 0.62f;
  car_body_trace_marker_.color.a = 0.2f;

  shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
  planning_state_marker_ = car_body_trace_marker_;
  planning_state_marker_.scale.z = m_scale * 6;

  planning_state_marker_.type = shape;
  planning_state_marker_.ns = "planning_state_marker";
  planning_state_marker_.scale.z = 0.5f;

  planning_state_marker_.color.r = 1.0f;
  planning_state_marker_.color.g = 1.0f;
  planning_state_marker_.color.b = 1.0f;
  planning_state_marker_.color.a = 1.0f;

  planning_bias_marker_ = planning_state_marker_;
  planning_bias_marker_.scale.z = m_scale * 8;
  planning_bias_marker_.ns = "planning_bias_marker";
}

bool ManagerMarker::UpdatePlanningMarker(const minieye::Planning& planning, float& body_width) {
  auto planning_markers_size = planning_markers_.markers.size();

  for (size_t index = 0; index < planning_markers_size; index++) {
    planning_markers_.markers[index].points.clear();
  }
  planning_marker_.points.clear();
  car_body_trace_marker_.points.clear();

  if (planning.planning_trajectory().trajectory_size() == 0) {
    return false;
  }

  planning_marker_.header.stamp = car_body_trace_marker_.header.stamp = ros::Time::now();

  auto& planning_trajectory = planning.planning_trajectory();

  planning_marker_.id = planning_trajectory.id();

  car_body_trace_marker_.scale.x = body_width;

  auto trajectory_size = planning_trajectory.trajectory_size();

  for (int i = 0; i < trajectory_size; i++) {
    geometry_msgs::Point pt;
    pt.x = planning_trajectory.trajectory(i).path_point().x();
    pt.y = planning_trajectory.trajectory(i).path_point().y();
    pt.z = planning_trajectory.trajectory(i).path_point().z();
    planning_marker_.points.push_back(pt);
    car_body_trace_marker_.points.push_back(pt);
  }

  auto end_point = planning_marker_.points[trajectory_size - 1];
  planning_bias_marker_.pose.position.x = end_point.x;
  planning_bias_marker_.pose.position.y = end_point.y;
  planning_bias_marker_.pose.position.z = end_point.z;

  std::string target_yaw_bias = std::to_string(planning.target_yaw_bias());
  std::string target_l_bias = std::to_string(planning.target_l_bias());
  std::string target_s_bias = std::to_string(planning.target_s_bias());
  std::string planning_bias = "(" + target_yaw_bias + ", " + target_l_bias + ", " + target_s_bias + ")";
  planning_bias_marker_.text = planning_bias;

  planning_markers_.markers.push_back(planning_marker_);
  planning_markers_.markers.push_back(car_body_trace_marker_);
  planning_markers_.markers.push_back(planning_bias_marker_);

  return true;
}

#ifdef MVIZ_TDA4
bool ManagerMarker::UpdateSlotsRecommendAtt(const minieye::PlanningToHMI& planning_to_hmi) {
  // todo:显示剩余距离
  float remain_distance = planning_to_hmi.remain_distance();
  google::protobuf::Map<int, int> recommand_id_list;
  recommand_id_list = planning_to_hmi.park_id().id_list();

  for (size_t index = 0; index < parkingspace_markers_.markers.size(); index++) {
    auto& marker = parkingspace_markers_.markers[index];
    if (marker.ns != "parkingspace") continue;
    auto rmd_slot_iter = recommand_id_list.find(marker.id);

    if (rmd_slot_iter != recommand_id_list.end()) {
      if (rmd_slot_iter->second == 0) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }
      // else{
      //   marker.color.r = 0.81f;
      //   marker.color.g = 0.81;
      //   marker.color.b = 0.81f;
      //   ROS_ERROR_STREAM(" recommend to not slot:"<<marker.id);
      // }
    }
  }
  return true;
}

bool ManagerMarker::UpdateSlotsSelectedAtt(const minieye::APAStateControl& apa_state_control) {
  google::protobuf::Map<int, int> apa_state_control_settings;
  apa_state_control_settings = apa_state_control.settings();
  std::int32_t apa_select_slot_id = apa_state_control_settings[0];
  // ROS_WARN_STREAM("selected slots:" << apa_select_slot_id);
  if (apa_select_slot_id == -1) {
    ROS_WARN_STREAM("hmi faild to chosened slots");
    return false;
  }

  for (size_t index = 0; index < parkingspace_markers_.markers.size(); index++) {
    auto& marker = parkingspace_markers_.markers[index];
    if (marker.ns != "parkingspace") continue;
    if (marker.id == apa_select_slot_id) {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
    }
  }
  return true;
}
#endif

#ifdef MVIZ_J3
bool ManagerMarker::UpdateSlotsRecommendAtt(const minieye::PlanningToHMI& planning_to_hmi) {
  // todo:显示剩余距离
  float remain_distance = planning_to_hmi.remain_distance();
  std::map<int, int> recommand_id_list;
  for (auto i : planning_to_hmi.park_id().id_list()) {
    recommand_id_list[i.key()] = i.value();
  }

  for (size_t index = 0; index < parkingspace_markers_.markers.size(); index++) {
    auto& marker = parkingspace_markers_.markers[index];
    if (marker.ns != "parkingspace") continue;
    auto rmd_slot_iter = recommand_id_list.find(marker.id);

    if (rmd_slot_iter != recommand_id_list.end()) {
      if (rmd_slot_iter->second == 0) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }
      // else{
      //   marker.color.r = 0.81f;
      //   marker.color.g = 0.81;
      //   marker.color.b = 0.81f;
      //   ROS_ERROR_STREAM(" recommend to not slot:"<<marker.id);
      // }
    }
  }
  return true;
}

bool ManagerMarker::UpdateSlotsSelectedAtt(const minieye::APAStateControl& apa_state_control) {
  std::map<int, int> apa_state_control_settings;
  for (auto i : apa_state_control.settings()) {
    apa_state_control_settings[i.settingtype()] = i.value();
  }
  std::int32_t apa_select_slot_id = apa_state_control_settings[0];
  // ROS_WARN_STREAM("selected slots:" << apa_select_slot_id);
  if (apa_select_slot_id == -1) {
    ROS_WARN_STREAM("hmi faild to chosened slots");
    return false;
  }

  for (size_t index = 0; index < parkingspace_markers_.markers.size(); index++) {
    auto& marker = parkingspace_markers_.markers[index];
    if (marker.ns != "parkingspace") continue;
    if (marker.id == apa_select_slot_id) {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
    }
  }
  return true;
}
#endif
bool ManagerMarker::UpdatePlanningState(float res, float cur_grer) {
  std::string gear;
  if (cur_grer == 0) {
    gear = "other:";
  } else if (cur_grer == 1) {
    gear = "R:";
  } else if (cur_grer == 2) {
    gear = "D:";
  }

  if (planning_marker_.points.size() <= 0) {
    return false;
  }
  planning_state_marker_.pose.position.x = planning_marker_.points[0].x;
  planning_state_marker_.pose.position.y = planning_marker_.points[0].y;
  planning_state_marker_.text = gear + std::to_string(res);

  return true;
}

void ManagerMarker::InitUssParkingspaceMarker() {
  // todo::uss_parkingspace include id 、corner_corodinate、corner_id
  // 、center_line、stage?

  uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

  uss_parkingspace_marker_ = parkingspace_marker_;
  uss_parkingspace_marker_.ns = "uss_parkingspace";
  uss_parkingspace_marker_.type = shape;

  uss_parkingspace_marker_.color.r = 0.56f;
  uss_parkingspace_marker_.color.g = 0.933f;
  uss_parkingspace_marker_.color.b = 0.56f;
  uss_parkingspace_marker_.color.a = 1.0f;

  uss_parkingspace_marker_text_ = parkingspace_marker_text_;
  uss_parkingspace_marker_text_.ns = "uss_parkingspace_text";
  uss_parkingspace_center_line_marker_ = center_line_marker_;
  uss_parkingspace_center_line_marker_.ns = "uss_parkingspace_center_line";

  uss_parkingspace_markers_.markers.push_back(uss_parkingspace_marker_);
  uss_parkingspace_markers_.markers.push_back(uss_parkingspace_marker_text_);
  uss_parkingspace_markers_.markers.push_back(uss_parkingspace_center_line_marker_);
}

bool ManagerMarker::UpdateUssParkingspaceMarker(const perception::ParkingSpace& uss_parkingspace) {
  std::uint32_t uss_parkingspace_slots_size = uss_parkingspace.parkslots_size();
  ROS_DEBUG_STREAM("uss_parkingspace_slots_size:" << uss_parkingspace_slots_size);

  if (uss_parkingspace_slots_size == 0) {
    return false;
  }

  auto uss_parkingspace_markers_size = uss_parkingspace_markers_.markers.size();
  for (size_t index = 0; index < uss_parkingspace_markers_size; index++) {
    uss_parkingspace_markers_.markers.clear();
  }
  uss_parkingspace_marker_.points.clear();

  uss_parkingspace_marker_.header.stamp = ros::Time::now();

  for (size_t index = 0; index < uss_parkingspace_slots_size; index++) {
    const perception::ParkingSlot& parking_slot = uss_parkingspace.parkslots(index);
    auto corner_point_size = parking_slot.corner_pts_size();
    if (corner_point_size < 4) {
      ROS_INFO_STREAM("uss_parkpingspace slot size:" << corner_point_size);
      continue;
    }

    // todo:update until next slots
    uss_parkingspace_marker_.points.clear();
    uss_parkingspace_center_line_marker_.points.clear();

    std::uint32_t slot_id = parking_slot.id();
    uss_parkingspace_marker_.id = slot_id;
    uss_parkingspace_marker_text_.id = slot_id;
    uss_parkingspace_center_line_marker_.id = slot_id;

    if (parking_slot.occupied()) {
      uss_parkingspace_marker_text_.color.r = 1.0f;
      uss_parkingspace_marker_text_.color.g = 0.0f;
    }

    visualization_msgs::Marker corner_id_marker = parkingspace_marker_text_;
    corner_id_marker.ns = "uss_corner_text";

    ROS_DEBUG_STREAM("uss_parkingspace slot id :" << slot_id);
    for (auto corner_point_index = 0; corner_point_index < corner_point_size; corner_point_index++) {
      geometry_msgs::Point pt;
      pt.x = parking_slot.corner_pts(corner_point_index).odom_pt().x();
      pt.y = parking_slot.corner_pts(corner_point_index).odom_pt().y();
      uss_parkingspace_marker_.points.push_back(pt);

      ROS_DEBUG_STREAM("uss_parkingspace_slot pt.x:" << pt.x << ",uss_parkingspace_slot pt.y:" << pt.y);

      // todo:: show corner point id
      static std::uint64_t corner_id = 0;
      corner_id_marker.id = corner_id++;
      corner_id_marker.pose.position.x = pt.x;
      corner_id_marker.pose.position.y = pt.y;
      corner_id_marker.text = std::to_string(parking_slot.corner_pts(corner_point_index).id());
      uss_parkingspace_markers_.markers.push_back(corner_id_marker);
    }
    uss_parkingspace_marker_.points.push_back(uss_parkingspace_marker_.points.front());

    // todo: center line  for every uss_parkingspace_slot
    geometry_msgs::Point centerPoint;
    centerPoint.x = (uss_parkingspace_marker_.points[0].x + uss_parkingspace_marker_.points[3].x) * 0.5;
    centerPoint.y = (uss_parkingspace_marker_.points[0].y + uss_parkingspace_marker_.points[3].y) * 0.5;
    centerPoint.z = (uss_parkingspace_marker_.points[0].z + uss_parkingspace_marker_.points[3].z) * 0.5;
    uss_parkingspace_center_line_marker_.points.push_back(centerPoint);

    centerPoint.x = (uss_parkingspace_marker_.points[1].x + uss_parkingspace_marker_.points[2].x) * 0.5;
    centerPoint.y = (uss_parkingspace_marker_.points[1].y + uss_parkingspace_marker_.points[2].y) * 0.5;
    centerPoint.z = (uss_parkingspace_marker_.points[1].z + uss_parkingspace_marker_.points[2].z) * 0.5;
    uss_parkingspace_center_line_marker_.points.push_back(centerPoint);

    // todo: slots id corodinate for every slots
    uss_parkingspace_marker_text_.pose.position.x =
        (uss_parkingspace_marker_.points[0].x + uss_parkingspace_marker_.points[2].x) * 0.5;
    uss_parkingspace_marker_text_.pose.position.y =
        (uss_parkingspace_marker_.points[0].y + uss_parkingspace_marker_.points[2].y) * 0.5;

    std::string ps_type_text = "";
    switch (parking_slot.type()) {
      case 1:
        ps_type_text = "V";
        break;
      case 2:
        ps_type_text = "H";
        break;

      case 3:
        ps_type_text = "K";
        break;
      default:
        ps_type_text = "U";
        break;
    }

    uss_parkingspace_marker_text_.text = std::to_string(uss_parkingspace_marker_.id) + ps_type_text;

    ROS_DEBUG_STREAM("uss_parkingspace_marker_ point size:" << uss_parkingspace_marker_.points.size());
    uss_parkingspace_markers_.markers.push_back(uss_parkingspace_marker_);
    uss_parkingspace_markers_.markers.push_back(uss_parkingspace_marker_text_);
    uss_parkingspace_markers_.markers.push_back(uss_parkingspace_center_line_marker_);
  }

  return true;
}

bool ManagerMarker::UpdateParkingTargetMarker(const minieye::parking::ObjectTrackList& pb) {
  parking_target_markers_.markers.clear();
  for (auto& obj : pb.object_track_list()) {
    parking_target_marker_.points.clear();
    parking_target_marker_.id = obj.id();
    parking_target_marker_.lifetime = ros::Duration(mod_lifetime_ * 1e-3);
    parking_target_marker_text_.id = obj.id();
    parking_target_marker_text_.lifetime = ros::Duration(mod_lifetime_ * 1e-3);
    parking_target_marker_text_.text = std::to_string(obj.id());

    mviz_utils::Box3D box;
    box.center = {obj.odom_info().position_value().x(), obj.odom_info().position_value().y(), 0.0};
    box.yaw = obj.odom_info().heading_value();
    box.roll = 0.0;
    box.pitch = 0.0;
    box.length = obj.odom_info().size().x();
    box.width = obj.odom_info().size().y();
    box.height = obj.odom_info().size().z();

    mviz_utils::Point3D vertices[10];
    box3d_vertexs_.calculateBoxVertices(box, vertices);
    for (auto line_idx : box3d_vertexs_.Box3DSides) {
      geometry_msgs::Point pt;
      pt.x = vertices[line_idx.first].x;
      pt.y = vertices[line_idx.first].y;
      pt.z = vertices[line_idx.first].z;
      parking_target_marker_.points.push_back(pt);
      pt.x = vertices[line_idx.second].x;
      pt.y = vertices[line_idx.second].y;
      pt.z = vertices[line_idx.second].z;
      parking_target_marker_.points.push_back(pt);
    }
    parking_target_marker_text_.pose.position.x = vertices[8].x;
    parking_target_marker_text_.pose.position.y = vertices[8].y;
    parking_target_marker_text_.pose.position.z = vertices[8].z;
    parking_target_markers_.markers.push_back(parking_target_marker_);
    parking_target_markers_.markers.push_back(parking_target_marker_text_);
  }
  // test
  // parking_target_marker_.points.clear();
  // mviz_utils::Box3D box;
  // box.center = {15, 60, 0.0};
  // box.yaw = -2.64428926;
  // box.roll = 0.0;
  // box.pitch = 0.0;
  // box.length = 1.89;
  // box.width = 1.18;
  // box.height = 1.0;
  // mviz_utils::Point3D vertices[10];
  // box3d_vertexs_.calculateBoxVertices(box, vertices);
  // for (auto line_idx : box3d_vertexs_.Box3DSides) {
  //   geometry_msgs::Point pt;
  //   pt.x = vertices[line_idx.first].x;
  //   pt.y = vertices[line_idx.first].y;
  //   pt.z = vertices[line_idx.first].z;
  //   parking_target_marker_.points.push_back(pt);
  //   pt.x = vertices[line_idx.second].x;
  //   pt.y = vertices[line_idx.second].y;
  //   pt.z = vertices[line_idx.second].z;
  //   parking_target_marker_.points.push_back(pt);
  // }
  // parking_target_markers_.markers.push_back(parking_target_marker_);
  // test end
  return true;
}