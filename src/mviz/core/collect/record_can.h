
// Copyright 2023 MINIEYE
#ifndef RECORD_CAN__H_
#define RECORD_CAN_H_
#include <fstream>
#include <map>
#include <thread>
#include <vector>
#include<mutex>
#include<deque>
#include "glog/logging.h"
namespace collect_can {

class RecordCan {
 public:
  RecordCan(const std::string& addr, const std::string& config_path);

  RecordCan(const RecordCan&) = delete;
  RecordCan operator=(const RecordCan&) = delete;
  RecordCan(RecordCan&&) = delete;
  ~RecordCan();

  bool start(const std::string& currBatchIdDir);
  bool stop();
  void close();
  bool bWantStop_{false};
  bool bWantStart_{false};

  bool GetCanInitStatus(){return init_status_;}
  bool InitCanSocket();

 private:
  bool ReceiveDataThreadFunc();
  bool UnpacDataTheadFunc(char* buff, size_t data_len);

  std::string UnpacDataTheadFunc(const std::string& data);

  bool SaveCanData();
  bool CloseCanSocket();
  bool ReadConfig(const std::string& config_path);

 private:
  std::ofstream* pCanRecordStream_{nullptr};
  std::thread* pRecvCanDataThread_{nullptr};
  std::thread::id can_thread_id_;
  int can_socket_{0};
  int endpoint_id_{-1};
  bool init_status_{false};

  std::string addr_;
  std::string config_path_;
  std::string data_path_;
  std::vector<std::string> recv_datas_vec_;

  std::map<std::string, std::string> link_mapping_table_;

  std::mutex mutex_;
  std::deque<std::string> can_recv_buff_;

  /* data */
};

}  // namespace collect_can

#endif