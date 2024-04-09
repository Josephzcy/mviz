// Copyright [2022] - MiniEye INC.

#ifndef CONFIGURATION_MANAGER_H
#define CONFIGURATION_MANAGER_H

#include <QDebug>
#include <map>
#include <vector>

#include "l2_types.h"
#include "yaml-cpp/yaml.h"

namespace mviz::replay {

template <typename T>
bool GetValue(const YAML::Node& node, const std::string& param_name, T& output);

class ConfigurationManager {
 public:
  ConfigurationManager(const std::string& config_file);

  MvizReplayConfig GetConfig();

  std::map<std::string, TopicConfig> m_topic_config_map;

 private:
  MvizReplayConfig m_mviz_replay_config;
};

}  // namespace mviz::replay

#endif