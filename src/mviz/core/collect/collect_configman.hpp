//
// Created by nick on 23-2-13.
//
#ifndef DATA_COLLECT_LIBFLOW_CONFIG_MAN_HPP_
#define DATA_COLLECT_LIBFLOW_CONFIG_MAN_HPP_
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

#include "glog/logging.h"
#include "json.hpp"

namespace minieye::mviz::config {

class LibFlowConf {
 public:
  LibFlowConf() = default;
  LibFlowConf(const std::string& _port, const std::string& _ip, const std::string& _topic,
              const std::string& _tick_unit)
      : port(_port), ip(_ip), topic(_topic), tick_unit(_tick_unit) {}

  std::string port;
  std::string ip;
  std::string topic;
  std::string tick_unit;
};

// typedef std::map<std::string, std::map<std::string,std::string>>
// ModuleTopicMap;
class CDataCollectConfigMan {
  typedef std::vector<LibFlowConf> LibFlowCollectVec;

 public:
  CDataCollectConfigMan() = default;
  explicit CDataCollectConfigMan(int mode) { init(mode); }
  CDataCollectConfigMan(const std::string& config_path, const std::string& mviz_mode) { init(config_path, mviz_mode); }

  bool init(int mode) {
    std::string config_file = ((0 == mode) ? "apa_hil_config.json" : "apa_oncar_config.json");

    nlohmann::json mviz_json_config;

    std::fstream fs(config_file, std::ios::in);
    if (!fs.good()) {
      LOG(ERROR) << "open mviz config file fail!";
      exit(-1);
    }
    fs >> mviz_json_config;

    for (auto field : mviz_json_config.items()) {
      if (field.key() != "topic_config") {
        LOG(ERROR) << "topic_config field false, please check the config file";
        return false;
      }

      // 读取ip
      auto topic_infos = field.value();

      // auto topic_infos = field.value();
      std::string port, ip, topic, tick_unit;

      for (auto conf = topic_infos.begin(); conf != topic_infos.end(); conf++) {
        bool is_topic_enble = conf->at("enable");
        if (is_topic_enble) {
          port = conf->at("port");
          ip = conf->at("ip");
          topic = conf->at("topic");
          tick_unit = conf->at("tick_unit");
          LOG(INFO) << "[ " << port << " " << ip << " " << topic << tick_unit << " ]" << std::endl;
          lib_flow_collect_vec_.emplace_back(port, ip, topic, tick_unit);
        }
      }
    }
    return true;
  }

  bool init(const std::string& config_path, const std::string& mviz_mode) {
    nlohmann::json mviz_json_config;

    std::fstream fs(config_path, std::ios::in);
    if (!fs.good()) {
      LOG(ERROR) << "open mviz config file fail!";
      exit(-1);
    }
    LOG(INFO) << "read collect config: [" << config_path << "]" << std::endl;
    LOG(INFO) << "mviz mode: [" << mviz_mode << "]" << std::endl;
    fs >> mviz_json_config;

    std::string port, ip, topic, tick_unit;
    for (auto field : mviz_json_config.items()) {
      if (field.key() == "ip") {
        ip = field.value();
        continue;
      }

      if (field.key() == "topic_config") {
        auto topic_infos = field.value();

        for (auto conf = topic_infos.begin(); conf != topic_infos.end(); conf++) {
          bool is_topic_enble = conf->at("enable");
          if (is_topic_enble) {
            port = conf->at("port");
            topic = conf->at("topic");
            tick_unit = conf->at("tick_unit");

            if (topic == "camera_stitching" && mviz_mode != "oncar") {
              ip = "127.0.0.1";
              port = "24011";
              topic = "camera30";
            } else if (mviz_mode == "hil_pc") {
              // hil_pb topic ip 127.0.0.1
              ip = "127.0.0.1";
              // if (port == "2080") continue;  
            } else if (mviz_mode == "hil_board") {
              ip = "92.168.98.233";
            }
            // can ip:192.168.98.180、127.0.0.1
            if (port == "2080" && mviz_mode == "oncar") {
              ip = "192.168.98.180";
            }
            LOG(INFO) << "[ " << port << " " << ip << " " << topic << " " << tick_unit << " ]" << std::endl;
            lib_flow_collect_vec_.emplace_back(port, ip, topic, tick_unit);
          }
        }
      }
    }
    return true;
  }

  LibFlowCollectVec& get_libflow_conf() { return lib_flow_collect_vec_; }

 private:
  LibFlowCollectVec lib_flow_collect_vec_;
  int mode_;
};

}  // namespace minieye::mviz::config
#endif
