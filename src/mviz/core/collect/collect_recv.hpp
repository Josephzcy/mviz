//
// Created by nick on 23-2-16.
//

#ifndef DATA_COLLECT_RECV_HPP_
#define DATA_COLLECT_RECV_HPP_
#include <flow.hpp>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <utility>

#include "collect_dbm.hpp"

namespace minieye::mviz::data {

class CollectFlowRecv : public flow::Client {
 public:
  CollectFlowRecv(const std::string &url, const std::string &topic)
      : flow::Client(flow::ClientConfig({"image", std::string("ws://") + url, topic})) {
    p_fLow_context = nullptr;
    sLocalUrl = url;
    sLocalTopic = topic;
    bStop = false;
    count = 0;
    bWant = true;
  }

  ~CollectFlowRecv() override {
    if (p_fLow_context != nullptr) {
      remove();
      p_fLow_context->stop();  // 析构时候需要stop
      delete p_fLow_context;
      p_fLow_context = nullptr;
      LOG(INFO) << "recv url[ " << sLocalUrl << "] topic: [ " << sLocalTopic << " ] closed " << std::endl;
    }
    bStop = true;
  }

  bool init() {
    if (sLocalUrl.empty()) {
      return false;
    }

    if (p_fLow_context != nullptr) {
      return false;
    }

    p_fLow_context = new flow::Context(flow::Config({{"servers", "none"}, {"sender.serialize", "raw"}}));
    p_fLow_context->add_client(this);

    if (p_fLow_context->start() != 0) {
      delete p_fLow_context;
      p_fLow_context = nullptr;
      LOG(ERROR) << "recv url[ " << sLocalUrl << "] topic: [ " << sLocalTopic << " ] error !!! ";
      return false;
    }
    LOG(INFO) << "recv url[ " << sLocalUrl << "] topic: [ " << sLocalTopic << " ] succeed ";

    return true;
  }
  // close 时候 remove
  void remove() {
    if (p_fLow_context != nullptr) {
      p_fLow_context->remove_client(this);
    }
  }

  // run 时候 add
  void add() {
    if (p_fLow_context != nullptr) {
      p_fLow_context->add_client(this);
    }
  }

  inline void logRecvTheData(bool bWant, const char *topicName, int no, int size, const char *data) {
    if (bWant) {
      std::stringstream ss;
      for (int i = 0; i < size; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << int(*(data + i));
      }
      LOG(INFO) << "LibflowClient: [" << topicName << "] [" << no << "] size:" << size << " data :" << ss.str();
    }
  }

  void recv(const char *source,  // '\0' terminated string
            const char *topic,   // any binary data
            const char *data,    // any binary data
            size_t size) override {
    // fprintf(stderr, "LibflowClient[ %s ]::recv(%s, %s, %s)\n", topic,
    //        source, topic, std::string(data, size).c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    if (bStop) {
      return;
    }
    // logRecvTheData(bWant,sLocalUrl.c_str(),count,size,data);
    // 放入 数据写入硬盘保存 DB/ 放入 显示缓冲区
    LOG(INFO) << "topic: [ " << sLocalTopic << " ] size " << size << std::endl;
    collect::CDataCollectDBMan::get_instance()->insert_data(sLocalTopic, data, size);
  }

 private:
  int count;
  bool bWant;
  flow::Context *p_fLow_context;
  std::string sLocalUrl;
  std::string sLocalTopic;
  std::mutex mutex_;
  bool bStop;
};
}  // namespace minieye::mviz::data

#endif