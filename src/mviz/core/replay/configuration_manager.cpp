// Copyright [2022] - MiniEye INC.

#include "configuration_manager.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
// #include "glog/logging.h"

namespace mviz::replay {

template <typename T>
bool GetValue(const YAML::Node& node, const std::string& param_name, T& output) {
  if (!node[param_name].IsDefined()) {
    return false;
  }
  output = node[param_name].as<T>();
  return true;
}

TopicConfig GetTopicConfig(const YAML::Node& node) {
  TopicConfig tpcfg;
  GetValue(node, "topic", tpcfg.topic);
  GetValue(node, "data_json_path", tpcfg.data_json_path);
  GetValue(node, "sync_by", tpcfg.sync_by);
  return tpcfg;
}

ReaderManagerConfig GetReaderManagerConfig(const YAML::Node& node) {
  ReaderManagerConfig config;
  GetValue(node, "mviz_data_path", config.mviz_data_path);
  GetValue(node, "key_topic", config.key_topic);
  GetValue(node, "sync_dt_threshold_ms", config.sync_dt_threshold_ms);
  GetValue(node, "video_codecs_type", config.video_codecs_type);
  for (const auto& pair : node["topic_configs"]) {
    std::string key = pair.first.as<std::string>();
    YAML::Node value = pair.second;
    config.topic_configs.push_back(GetTopicConfig(value));
  }

  return config;
}

CameraSwitchingConfig GetCameraSwitchingConfig(const YAML::Node& node) {
  CameraSwitchingConfig config;
  GetValue(node, "image_front_position", config.image_front_position);
  GetValue(node, "image_rear_position", config.image_rear_position);
  GetValue(node, "image_left_position", config.image_left_position);
  GetValue(node, "image_right_position", config.image_right_position);

  return config;
}

PubCameraConfig GetPubCameraConfig(const YAML::Node& node) {
  PubCameraConfig pub_camera_cfg;
  GetValue(node, "topic", pub_camera_cfg.topic);
  GetValue(node, "camera_number", pub_camera_cfg.camera_number);
  GetValue(node, "camera_position", pub_camera_cfg.camera_position);
  return pub_camera_cfg;
}

PubManagerConfig GetPubManagerConfig(const YAML::Node& node) {
  PubManagerConfig config;
  GetValue(node, "car_body_dir", config.car_body_dir);
  GetValue(node, "marker_radio", config.marker_radio);
  GetValue(node, "marker_scale", config.marker_scale);
  for (const auto& pair : node["pub_camera_configs"]) {
    std::string key = pair.first.as<std::string>();
    YAML::Node value = pair.second;
    config.pub_camera_config_list.push_back(GetPubCameraConfig(value));
  }
  return config;
}

void printYamlNode(const YAML::Node& node) {
  QString yamlString = QString::fromStdString(YAML::Dump(node));
  qDebug() << yamlString;
}

ConfigurationManager::ConfigurationManager(const std::string& config_file) {
  // std::cout << "reading config from " << config_file << " :" << std::endl;
  // qDebug() << "reading config from " << QString::fromStdString(config_file);
  YAML::Node root_node = YAML::LoadFile(config_file);
  // printYamlNode(root_node);
  m_mviz_replay_config.reader_manager_config = GetReaderManagerConfig(root_node["reader_manager_config"]);
  m_mviz_replay_config.camera_stitching_config = GetCameraSwitchingConfig(root_node["camera_stitching_config"]);
  m_mviz_replay_config.pub_manager_config = GetPubManagerConfig(root_node["pub_manager_config"]);
}

MvizReplayConfig ConfigurationManager::GetConfig() { return m_mviz_replay_config; }

}  // namespace mviz::replay