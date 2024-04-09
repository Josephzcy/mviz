#ifndef DATA_COLLECT_SRV_HPP
#define DATA_COLLECT_SRV_HPP
#include <ros/ros.h>
#include <sys/time.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "collect/collect_producer.hpp"
#include "collect/record_can.h"
#include "topic_handle.hpp"
// libflow recved data
// class CCollectProducer;

template <typename T>
nlohmann::json minieye::mviz::topic::TopicManagerBase<T>::hmiTime_json = {};

namespace minieye::mviz::collect {
typedef std::map<std::string, CCollectProducer *> CollectPdrMap;
typedef enum { EMPTY, INITED, RUNNING, STOP } emSRV_STATE;

class CDataCollectSrv {
 public:
  static CDataCollectSrv *get_instance() { return pSingletonCollectSrv; }

  static void delete_instance() {
    if (pSingletonCollectSrv) {
      pSingletonCollectSrv->release_collect_map();
      delete pSingletonCollectSrv;
      pSingletonCollectSrv = nullptr;
    }
  }

  CDataCollectSrv(const CDataCollectSrv &) = delete;
  CDataCollectSrv(CDataCollectSrv &&) = delete;
  CDataCollectSrv &operator=(CDataCollectSrv const &) = delete;

 public:
  emSRV_STATE get_srv_state() { return emSvrState; }

  int load_params(std::vector<std::string> &topicVec) {
    if (get_srv_state() > EMPTY) {
      LOG(ERROR) << "loaded params error : had been ";
      return -1;
    }
    // must call ros::init() before creating the NodeHandle
    ros::NodeHandle nh_;
    std::string car_body_dir;
    nh_.param<bool>("/save_mviz_data", bSaveMvizData_, true);
    nh_.param<std::string>("/save_mviz_data_path", mviz_data_path_, "/root/mviz_apa/mviz_data/");
    nh_.param<std::string>("/car_body_dir", car_body_dir, "/root/mviz/config/");

    std::string can_config_path = car_body_dir.substr(0, car_body_dir.find_last_of('/')) + "/" + "can_link_config.yaml";
    std::cout << "[Info:]"
              << "can_config_path:===" << can_config_path << std::endl;

    LOG(INFO) << "====bSaveMvizData_ [" << bSaveMvizData_ << "]";
    LOG(INFO) << "====car_body_dir [" << car_body_dir << "]";

    // 获取车型的名字并加载对应的can配置
    auto end = mviz_data_path_.length() - 1;
    if (mviz_data_path_[end] != '/') {
      mviz_data_path_.push_back('/');
    }

    std::string can_addr;
    for (const auto &conf : collect_config_man_.get_libflow_conf()) {
      if (conf.topic == "*") {
        can_addr = "tcp://" + conf.ip + ":" + conf.port;
        continue;  // can port data
      }
      reg_producers_and_bind_cbs(conf);
      topicVec.push_back(conf.topic);
    }
    LOG(INFO) << "mviz collect srv load module params succeed" << std::endl;

    LOG(INFO) << "can_addr:" << can_addr << std::endl;
    LOG(INFO) << "can_config_path:" << can_config_path << std::endl;

    if (!can_addr.empty()) {  // enable can_collect
      if (pRecordcanData == nullptr) pRecordcanData = new collect_can::RecordCan(can_addr, can_config_path);
    }

    set_srv_state(INITED);
    return 0;
  }

  int start() {
    if (get_srv_state() < INITED || get_srv_state() == RUNNING) return -1;

    get_cur_batch_id_dir();

    if (bSaveMvizData_) {
      if (!create_batch_dir()) return -1;
    }

    for (auto &it : collect_pdr_map_) {
      it.second->_start(currBatchIdDir.str().c_str());
    }
    static bool have_inited = false;
    if (pRecordcanData != nullptr) {  // have inited
      if (!pRecordcanData->GetCanInitStatus()) {
        if (pRecordcanData->InitCanSocket()) have_inited = true;
      }
      if (have_inited) pRecordcanData->start(currBatchIdDir.str());
    }
    
    set_srv_state(RUNNING);
    return 0;
  }

  int stop() {
    if (get_srv_state() != RUNNING) {
      return -1;
    }
    // hmiTime.json
    nlohmann::json temp = minieye::mviz::topic::TopicManagerBase<std::string>::hmiTime_json;
    SaveToJson(temp, currBatchIdDir.str(), "hmiTime.json");
    for (auto &it : collect_pdr_map_) {
      it.second->_close();
    }

    if (pRecordcanData != nullptr) pRecordcanData->stop();

    set_srv_state(STOP);
    return 0;
  }

  void release_collect_map() {
    stop();
    for (auto &it : collect_pdr_map_) {
      delete it.second;
      it.second = nullptr;
    }
    collect_pdr_map_.clear();

    if (pRecordcanData != nullptr) {
      delete pRecordcanData;
      pRecordcanData = nullptr;
    }
  }

  config::CDataCollectConfigMan &get_collect_config_man() { return collect_config_man_; }

 private:
  CDataCollectSrv() noexcept : emSvrState(EMPTY) {}

  void set_srv_state(emSRV_STATE tmp) { emSvrState = tmp; }

  int reg_producers_and_bind_cbs(const config::LibFlowConf &conf) {
    auto ret = 0;
    auto it = collect_pdr_map_.find(conf.topic);
    if (it != collect_pdr_map_.end()) {
      ret = -1;
    } else {
      auto pCollectRecProducer = new CCollectProducer(conf, &collect_config_man_, bSaveMvizData_);
      collect_pdr_map_.insert(std::make_pair(conf.topic, pCollectRecProducer));
      LOG(INFO) << "add new topic producer: [" << conf.topic << "]" << std::endl;
      ret = 0;
    }

    return ret;
  }

  void get_cur_batch_id_dir() {
    char timebuf[64] = {0x00};
    currBatchIdDir.str("");
    currBatchIdDir.clear();
    struct timeval tv {};
    gettimeofday(&tv, nullptr);
    struct tm *lt = localtime(&tv.tv_sec);
    snprintf(timebuf, sizeof(timebuf) - 1,
             "%d%02d%02d%02d%02d%02d",  //"%d%02d%02d%02d%02d%02d%03d",
             lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday, lt->tm_hour, lt->tm_min,
             lt->tm_sec);  // static_cast<int>((tv.tv_usec / 1000)));
    currBatchIdDir << mviz_data_path_ << timebuf;
    LOG(INFO) << "mviz data saved path: " << (mviz_data_path_ + timebuf) << " " << __FUNCTION__ << std::endl;
  }

  bool create_batch_dir() {
    boost::filesystem::path batchIdDir(currBatchIdDir.str().c_str());
    return boost::filesystem::create_directories(batchIdDir);
  }

  void SaveToJson(nlohmann::json &minieye_obj, std::string dst_dir, const std::string file_name) {
    std::string target_file = dst_dir + "/" + file_name;
    std::cout << "save:" << target_file << std::endl;
    std::ofstream destFile(target_file, std::ios::out);
    destFile << std::setw(4) << minieye_obj << std::endl;
    destFile.close();
  }

 private:
  std::stringstream currBatchIdDir;
  config::CDataCollectConfigMan collect_config_man_;
  CollectPdrMap collect_pdr_map_;
  emSRV_STATE emSvrState;

  collect_can::RecordCan *pRecordcanData{nullptr};

  static CDataCollectSrv *pSingletonCollectSrv;

  bool bSaveMvizData_;
  std::string mviz_data_path_;
};
}  // namespace minieye::mviz::collect
#endif
