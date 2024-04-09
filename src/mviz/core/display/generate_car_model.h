
// Copyright 2023 MINIEYE
#ifndef GENERATE_CAR_MODEL_H_
#define GENERATE_CAR_MODEL_H_
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

class GenerateCarModel {
 public:
  using LineSegment = std::pair<geometry_msgs::Point, geometry_msgs::Point>;
  GenerateCarModel(const std::string& car_config_path, const float& marker_radio, const float& marker_scale);
  ~GenerateCarModel();
  GenerateCarModel(const GenerateCarModel&) = delete;
  GenerateCarModel& operator=(const GenerateCarModel&) = delete;
  GenerateCarModel(GenerateCarModel&&) = delete;
  GenerateCarModel& operator=(GenerateCarModel&&) = delete;

  void MakeCarModel(visualization_msgs::MarkerArray& car_model);
  void InitCarModelCorodinate();

  std::string m_carConfigPath;
  std::string m_vehicle_plaform;
  float m_car_model_radio;

  float m_length;
  float m_width;
  float m_height;

  float m_wheel_base;
  float m_front_wheel_distance;
  float m_rear_wheel_distance;
  float m_front_suspension_length;
  float m_rear_suspension_length;
  float m_car_scale;
  float m_safety_d;
  std::uint16_t m_step_x;
  std::uint16_t m_step_y;

 private:
  std::map<std::string, LineSegment> car_model_cordinate_map_;
  std::map<std::string, LineSegment> car_model_safety_map_;

  visualization_msgs::Marker car_model;
  geometry_msgs::Point m_initPosition;
};
#endif
