#ifndef MVIZ_COLLECT_PRODUCER_HPP
#define MVIZ_COLLECT_PRODUCER_HPP

#include <iostream>
#include <thread>

#include "collect_configman.hpp"
#include "collect_dbm.hpp"
#include "collect_recv.hpp"
#include "file_sys_record.hpp"

namespace minieye::mviz::collect {

class CCollectProducer {
 public:
  CCollectProducer(const config::LibFlowConf &conf, config::CDataCollectConfigMan *config, bool save_data) noexcept {
    bWantStop = true;
    producerThead = nullptr;
    bStart = false;
    pFileSysRecord = nullptr;
    producerName = conf.topic;
    url = conf.ip + ":" + conf.port;
    p_collect_config_man_ = config;
    pFlowRecvWork = nullptr;
    bFlowRecvWorkerRunning = false;
    bSaveData = save_data;
    tick_unit = conf.tick_unit;
  }
  CCollectProducer(const CCollectProducer &) = delete;
  CCollectProducer(CCollectProducer &&) = delete;
  ~CCollectProducer() {
    if (pFlowRecvWork != nullptr) {  // PUB ROS MSG
      delete pFlowRecvWork;
      pFlowRecvWork = nullptr;
    }
    _close();
  }
  bool _start(const char *batchIdDir) {
    if (producerThead != nullptr) return false;

    strCurBatchIdDir.clear();
    strCurBatchIdDir = batchIdDir;

    producerThead = new std::thread(&CCollectProducer::producer_thread_func, this);
    // waitForSingeObject();
    threadId = producerThead->get_id();
    LOG(INFO) << "pdr_name:[ " << producerName << " ] - id:[ " << threadId << " ] is running...";
    return bStart;
  }
  void _close() {
    bWantStop = true;
    if (producerThead != nullptr) {
      producerThead->join();
      delete producerThead;
      producerThead = nullptr;
      LOG(INFO) << "pdr_name: [ " << producerName << "] -id: [ " << threadId << " ] was exited...";
    }
    bStart = false;
  }

 private:
  //=========load & UnLoad TopicRecvWork ======
  int load_topic_func() {
    if (pFlowRecvWork == nullptr) {
      pFlowRecvWork = new data::CollectFlowRecv(url, producerName);
    }

    if (!bFlowRecvWorkerRunning) {
      if (!pFlowRecvWork->init()) {
        delete pFlowRecvWork;
        pFlowRecvWork = nullptr;
        return -2;
      }
      bFlowRecvWorkerRunning = true;
    } else {
      pFlowRecvWork->add();
    }

    return 0;
  }
  void unload_topic_func() {
    if (pFlowRecvWork != nullptr) {
      pFlowRecvWork->remove();
    }
  }
  //=========load & UnLoad TopicRecvWork ======

  //==========load & UnLoad FileSysRecord ======
  int load_file_record_func() {
    if (pFileSysRecord == nullptr) {
      pFileSysRecord = new file::CFileSysRecord(strCurBatchIdDir.c_str(), producerName.c_str(),tick_unit);
      if (!pFileSysRecord->_start()) {
        delete pFileSysRecord;
        pFileSysRecord = nullptr;
        return -2;
      }
      return 0;
    } else {
      LOG(ERROR) << "pdr_name:[ " << producerName << " ] load file record setup ERROR";
    }
    return -1;
  };

  void unload_file_record_func() {
    if (pFileSysRecord != nullptr) {
      delete pFileSysRecord;
      pFileSysRecord = nullptr;
    }
  };
  //==========load & UnLoad FileSysRecord ======

  void producer_thread_func() {
    bStart = true;
    bWantStop = false;

    if (bSaveData) {
      // 文件保存系统开启
      if (load_file_record_func() == 0)
        LOG(INFO) << "pdr_name:[ " << producerName << " ] file record setup OK";
      else
        LOG(ERROR) << "pdr_name:[ " << producerName << " ] file record setup FAILED";
    }
    // 安装 TOPIC 接收
    if (load_topic_func() == 0)
      LOG(INFO) << "pdr_name:[ " << producerName << " ] topic setup OK";
    else
      LOG(ERROR) << "pdr_name:[ " << producerName << " ] topic setup FAILED";

    while (!bWantStop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      result = "2";
    }

    // 卸载TOPIC 接收
    unload_topic_func();
    LOG(INFO) << "pdr_name:[ " << producerName << " ] uninstall topic recv";
    // 卸载文件系统，并关闭保存
    unload_file_record_func();
    LOG(INFO) << "pdr_name:[ " << producerName << " ] uninstall file record";
    LOG(INFO) << "pdr_name:[ " << producerName << " ] -id: [ " << threadId << " ] is exit...";
  }
  bool bWantStop;
  bool bStart;
  std::thread *producerThead;
  std::thread::id threadId;
  config::CDataCollectConfigMan *p_collect_config_man_;
  std::string producerName;
  std::string url;
  data::CollectFlowRecv *pFlowRecvWork;
  file::CFileSysRecord *pFileSysRecord;
  std::string strCurBatchIdDir;
  bool bFlowRecvWorkerRunning;
  std::string result;
  bool bSaveData;
  std::string tick_unit{"us"};
};

}  // namespace minieye::mviz::collect

#endif  // MVIZ_COLLECT_PRODUCER_HPP
