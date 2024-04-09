#include "generate_car_model.h"

#include <tf/transform_broadcaster.h>

#include <iostream>

#include "yaml-cpp/yaml.h"

GenerateCarModel::GenerateCarModel(const std::string& car_config_path, const float& marker_radio,
                                   const float& marker_scale) {
  m_initPosition.x = m_initPosition.y = m_initPosition.z = 0.0f;

  YAML::Node config = YAML::LoadFile(car_config_path);
  m_vehicle_plaform = config["vehicle_plaform"].as<std::string>();

  m_car_model_radio = marker_radio;
  m_car_scale = marker_scale;

  m_length = config["length"].as<float>() * m_car_model_radio;
  m_width = config["width"].as<float>() * m_car_model_radio;
  m_height = config["height"].as<float>() * m_car_model_radio;

  m_wheel_base = config["wheel_base"].as<float>() * m_car_model_radio;
  m_front_wheel_distance = config["front_wheel_distance"].as<float>() * m_car_model_radio;
  m_rear_wheel_distance = config["rear_wheel_distance"].as<float>() * m_car_model_radio;
  m_front_suspension_length = config["front_suspension_length"].as<float>() * m_car_model_radio;
  m_rear_suspension_length = config["rear_suspension_length"].as<float>() * m_car_model_radio;

  m_safety_d = config["safety_d"].as<float>() * m_car_model_radio;
  m_step_x = config["step_x"].as<std::uint16_t>() * m_car_model_radio;
  m_step_y = config["step_y"].as<std::uint16_t>() * m_car_model_radio;

  InitCarModelCorodinate();
}

GenerateCarModel::~GenerateCarModel() {
  car_model_cordinate_map_.clear();
  car_model_safety_map_.clear();
}

void GenerateCarModel::InitCarModelCorodinate() {
  // todo:安全区域和车模轮廓
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  LineSegment segment;

  segment.first.x = m_initPosition.x + m_wheel_base + m_front_suspension_length;
  segment.first.y = m_initPosition.y + 0.5 * m_width;
  segment.first.z = m_initPosition.z;

  segment.second.x = m_initPosition.x - m_rear_suspension_length;
  segment.second.y = m_initPosition.y + 0.5 * m_width;
  segment.second.z = m_initPosition.z;

  car_model_cordinate_map_["left_line"] = segment;  // todo: left_line

  segment.second.x = m_initPosition.x - m_rear_suspension_length;
  segment.second.y = m_initPosition.y + 0.5 * m_width;
  segment.second.z = m_initPosition.z;

  segment.first.x = m_initPosition.x - m_rear_suspension_length;
  segment.first.y = m_initPosition.y - 0.5 * m_width;
  segment.first.z = m_initPosition.z;

  car_model_cordinate_map_["down_line"] = segment;  // todo: down_line

  segment.second.x = m_initPosition.x - m_rear_suspension_length;
  segment.second.y = m_initPosition.y - 0.5 * m_width;
  segment.second.z = m_initPosition.z;

  segment.first.x = m_initPosition.x + m_wheel_base + m_front_suspension_length;
  segment.first.y = m_initPosition.y + -0.5 * m_width;
  segment.first.z = m_initPosition.z;

  car_model_cordinate_map_["right_line"] = segment;  // todo: right_line

  segment.second.x = m_initPosition.x + m_wheel_base + m_front_suspension_length;
  segment.second.y = m_initPosition.y + 0.5 * m_width;
  segment.second.z = m_initPosition.z;

  segment.first.x = m_initPosition.x + m_wheel_base + m_front_suspension_length;
  segment.first.y = m_initPosition.y - 0.5 * m_width;
  segment.first.z = m_initPosition.z;

  car_model_cordinate_map_["up_line"] = segment;  // todo: up_line

  // todo线段内部
  segment.first.x = m_initPosition.x - m_rear_suspension_length;
  segment.first.y = m_initPosition.y;
  segment.first.z = m_initPosition.z;

  segment.second.x = m_initPosition.x + m_wheel_base + m_front_suspension_length;
  segment.second.y = m_initPosition.y;
  segment.second.z = m_initPosition.z;
  car_model_cordinate_map_["axis_line"] = segment;  // todo: Longitudinal axis line

  segment.second.x = m_initPosition.x + m_wheel_base;
  segment.second.y = m_initPosition.y + 0.5 * m_width;
  segment.second.z = m_initPosition.z;

  segment.first.x = m_initPosition.x + m_wheel_base + m_front_suspension_length;
  segment.first.y = m_initPosition.y;
  segment.first.z = m_initPosition.z;

  car_model_cordinate_map_["lf_line"] = segment;  // todo: left_front line

  segment.first.x = m_initPosition.x + m_wheel_base;
  segment.first.y = m_initPosition.y - 0.5 * m_width;
  segment.first.z = m_initPosition.z;

  segment.second.x = m_initPosition.x + m_wheel_base + m_front_suspension_length;
  segment.second.y = m_initPosition.y;
  segment.second.z = m_initPosition.z;
  car_model_cordinate_map_["rf_line"] = segment;  // todo: right_front line

  segment.first.x = m_initPosition.x + m_wheel_base;
  segment.first.y = m_initPosition.y + 0.5 * m_width;
  segment.first.z = m_initPosition.z;

  segment.second.x = m_initPosition.x + m_wheel_base;
  segment.second.y = m_initPosition.y - 0.5 * m_width;
  segment.second.z = m_initPosition.z;
  car_model_cordinate_map_["fw_line"] = segment;  // todo: front_wheel line

  segment.first.x = m_initPosition.x;
  segment.first.y = m_initPosition.y + 0.5 * m_width;
  segment.first.z = m_initPosition.z;

  segment.second.x = m_initPosition.x;
  segment.second.y = m_initPosition.y - 0.5 * m_width;
  segment.second.z = m_initPosition.z;
  car_model_cordinate_map_["rw_line"] = segment;  // todo: rear_wheel line

  // todo:主车安全距离,five points ,show using dotted line

  // todo: left safety line
  float height = m_length + 2 * m_safety_d;
  height = height - height / m_step_x * 0.5;

  float width = m_width + 2 * m_safety_d;
  width = width - width / m_step_y * 0.5;

  segment.first.x = m_initPosition.x + m_wheel_base + m_front_suspension_length + m_safety_d;
  segment.first.y = m_initPosition.y + 0.5 * m_width + m_safety_d;
  segment.first.z = m_initPosition.z;

  segment.second.x = m_initPosition.x + m_wheel_base + m_front_suspension_length + m_safety_d - height / m_step_x * 0.5;
  segment.second.y = m_initPosition.y + 0.5 * m_width + m_safety_d;
  segment.second.z = m_initPosition.z;

  car_model_safety_map_["left_safe_line"] = segment;

  for (auto step_index = 0; step_index < m_step_x; step_index++) {
    segment.first.x = segment.first.x - height / m_step_x;

    segment.second.x = segment.second.x - height / m_step_x;

    std::string key = "left_safeline_" + std::to_string(step_index);
    car_model_safety_map_[key] = segment;
  }

  // todo: for down_line
  segment.first = segment.second;
  segment.second.x = segment.first.x;
  segment.second.y = segment.first.y - width / m_step_y * 0.5;
  segment.second.z = m_initPosition.z;
  car_model_safety_map_["down_safeline"] = segment;
  for (auto step_index = 0; step_index < m_step_y; step_index++) {
    segment.first.y = segment.first.y - width / m_step_y;

    segment.second.y = segment.second.y - width / m_step_y;
    std::string key = "down_safeline_" + std::to_string(step_index);
    car_model_safety_map_[key] = segment;
  }

  // todo: for right_line
  segment.first = segment.second;
  segment.second.x = segment.first.x + height / m_step_x * 0.5;
  segment.second.y = segment.first.y;

  segment.second.z = m_initPosition.z;
  car_model_safety_map_["right_safeline"] = segment;

  for (auto step_index = 0; step_index < m_step_x; step_index++) {
    segment.first.x = segment.first.x + height / m_step_x;

    segment.second.x = segment.second.x + height / m_step_x;
    std::string key = "right_safeline_" + std::to_string(step_index);
    car_model_safety_map_[key] = segment;
  }

  // todo: for up_line
  segment.first = segment.second;
  segment.second.x = segment.first.x;
  segment.second.y = segment.first.y + width / m_step_y * 0.5;
  segment.second.z = m_initPosition.z;
  car_model_safety_map_["up_safe_line"] = segment;

  for (auto step_index = 0; step_index < m_step_y; step_index++) {
    segment.first.y = segment.first.y + width / m_step_y;

    segment.second.y = segment.second.y + width / m_step_y;
    std::string key = "up_safeline_" + std::to_string(step_index);
    car_model_safety_map_[key] = segment;
  }
}
void GenerateCarModel::MakeCarModel(visualization_msgs::MarkerArray& car_model_maker) {
  visualization_msgs::Marker car_body_marker;
  visualization_msgs::Marker safety_dis_marker;
  car_body_marker.header.frame_id = "base_link";
  car_body_marker.header.stamp = ros::Time(0);
  car_body_marker.ns = "car_model";
  car_body_marker.action = visualization_msgs::Marker::ADD;
  car_body_marker.id = 1;
  car_body_marker.type = visualization_msgs::Marker::LINE_LIST;

  tf::Quaternion q;
  q.setEuler(0, 0, 0);
  car_body_marker.pose.position.x = m_initPosition.x;
  car_body_marker.pose.position.y = m_initPosition.y;
  car_body_marker.pose.position.z = m_initPosition.z;
  car_body_marker.pose.orientation.x = q[0];
  car_body_marker.pose.orientation.y = q[1];
  car_body_marker.pose.orientation.z = q[2];
  car_body_marker.pose.orientation.w = q[3];

  // todo:车模为green,scale 线宽
  car_body_marker.scale.x = m_car_scale;
  car_body_marker.color.r = 1.0f;
  car_body_marker.color.g = 0.89f;
  car_body_marker.color.b = 0.7f;
  car_body_marker.color.a = 1.0f;

  for (auto& segement : car_model_cordinate_map_) {
    car_body_marker.points.emplace_back(segement.second.first);
    car_body_marker.points.emplace_back(segement.second.second);
  }

  safety_dis_marker = car_body_marker;
  safety_dis_marker.points.clear();  // todo::clear car_body_marker data

  safety_dis_marker.id = 2;
  // todo:车模为green,scale 线宽
  safety_dis_marker.color.r = 1.0f;
  safety_dis_marker.color.g = 1.0f;
  safety_dis_marker.color.b = 0.0f;
  safety_dis_marker.color.a = 1.0f;

  safety_dis_marker.type = visualization_msgs::Marker::LINE_LIST;
  for (auto& segement : car_model_safety_map_) {
    safety_dis_marker.points.emplace_back(segement.second.first);
    safety_dis_marker.points.emplace_back(segement.second.second);
  }
  car_model_maker.markers.push_back(car_body_marker);
  car_model_maker.markers.push_back(safety_dis_marker);
}