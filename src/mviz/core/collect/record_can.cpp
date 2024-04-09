
#include "record_can.h"

#include <math.h>  //modf()

#include <chrono>
#include <iostream>
#include <sstream>

#include "nanomsg/nn.h"
#include "nanomsg/pubsub.h"
#include "type_conversion.hpp"
#include "yaml-cpp/yaml.h"

namespace collect_can {

RecordCan::RecordCan(const std::string& addr, const std::string& config_path) : addr_(addr), config_path_(config_path) {
  if (config_path.empty()) {
    std::cout << "Error: can config path is empty" << std::endl;
  }

  if (ReadConfig(config_path_)) {
    std::cout << "Error: read config path success" << std::endl;
  }
  // InitCanSocket();
}

bool RecordCan::ReadConfig(const std::string& config_path) {
  if (config_path.empty()) return false;
  YAML::Node config = YAML::LoadFile(config_path);
  link_mapping_table_["can1"] = config["can1"].as<std::string>();
  link_mapping_table_["can2"] = config["can2"].as<std::string>();
  link_mapping_table_["can3"] = config["can3"].as<std::string>();
  link_mapping_table_["can4"] = config["can4"].as<std::string>();
  link_mapping_table_["can5"] = config["can5"].as<std::string>();
  link_mapping_table_["can6"] = config["can6"].as<std::string>();
  link_mapping_table_["can7"] = config["can7"].as<std::string>();
  link_mapping_table_["can8"] = config["can8"].as<std::string>();

  return true;
}
bool RecordCan::InitCanSocket() {
  can_socket_ = nn_socket(AF_SP, NN_SUB);

  if (can_socket_ < 0) {
    std::cout << "[Eroor]: can nn_socket error:" << nn_strerror(errno) << std::endl;
    return false;
  }

  // subscribe all topoic data
  if (nn_setsockopt(can_socket_, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0) {
    std::cout << "[Error]: setting subscription:" << nn_strerror(nn_errno()) << std::endl;
    nn_close(can_socket_);
    return false;
  }

  int timeout = 3000;  // for 5s
  if (nn_setsockopt(can_socket_, NN_SOL_SOCKET, NN_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    std::cout << "[Error]: setting receive timeout:" << nn_strerror(nn_errno()) << std::endl;
    nn_close(can_socket_);
    return false;
  }

  const std::string url = addr_;
  endpoint_id_ = nn_connect(can_socket_, url.c_str());
  if (endpoint_id_ < 0) {
    if ((nn_errno() == ETIMEDOUT)) {
      std::cout << "[Error]:receive timeout" << std::endl;
    } else {
      std::cout << "[Error]: connect error" << std::endl;
    }
    nn_close(can_socket_);
    return false;
  }

  std::cout << "[Info]:can_socket connect success" << std::endl;
  init_status_ = true;
  return true;
}

bool RecordCan::ReceiveDataThreadFunc() {
  while (!bWantStop_) {
    char* buff = nullptr;
    int recv_data_len = nn_recv(can_socket_, &buff, NN_MSG, 0);
    if (recv_data_len < 0) {
      if ((nn_errno() == ETIMEDOUT)) {
        std::cout << "[Error]:receive timeout" << std::endl;
      } else {
        std::cout << "[Error]: nn_recv data is empty" << std::endl;
      }
      return false;
    }

    recv_datas_vec_.emplace_back(std::string(buff, recv_data_len));
    // todo:注意这里解包的数据量少，应该不会发送过去导致数据丢失的问题
    nn_freemsg(buff);
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    // {
    //   std::lock_guard<std::mutex> lock(mutex_); // auto unlock
    //   can_recv_buff_.emplace_back(std::string(buff, recv_data_len));
    //   // todo:注意这里解包的数据量少，应该不会发送过去导致数据丢失的问题
    //   nn_freemsg(buff);
    //   std::this_thread::sleep_for(std::chrono::microseconds(10));
    // }
  }

  // todo::获取本次要存的数据路径和存储接收的时候
  SaveCanData();

  return true;
}

bool RecordCan::UnpacDataTheadFunc(char* buff, size_t data_len) {
  std::string recv_single_can(buff, data_len);
  if (data_len <= 0) return false;

  std::string timestamp = recv_single_can.substr(8, 8);
  std::string channel = recv_single_can.substr(2, 1);
  std::string can_id = recv_single_can.substr(4, 4);                 // hex
  std::string can_data = recv_single_can.substr(16, data_len - 16);  // hex

  auto ts = *((double*)timestamp.c_str());

  std::string timestamp_s, timestamp_ns;
  mviz_record::type_conversion::SplitFloat(ts, timestamp_s, timestamp_ns);
  timestamp = timestamp_s + " " + timestamp_ns;
  int channel_int = static_cast<int>(*channel.c_str()) + 1;  // net is 0-7 but can_link is 1-8

  std::string channel_str = std::to_string(channel_int);
  std::string can_link = "can" + channel_str;  // can_mapping_table
  std::string key_world = "bm.0.can" + channel_str + "." + link_mapping_table_.at(can_link);
  std::string can_id_hex = mviz_record::type_conversion::int_to_hex(*(std::uint32_t*)(can_id.c_str()));
  std::string can_data_hex = mviz_record::type_conversion::string_to_hex(can_data);

  // std::cout << "[Info]:timestamp_s: " << timestamp_s << ",timestamp_ns:" << timestamp_ns << std::endl;
  // std::cout << "[Info]:key_world: " << key_world << std::endl;
  // std::cout << "[Info]:channnel: " << channel_int << std::endl;
  // std::cout << "[Info]:can_id: " << can_id_hex << std::endl;
  // std::cout << "[Info]:can_data: " << can_data_hex << std::endl;

  // mviz_can.txt: s、us、key_eorld、can-id(ox+hex)、can-data(hex)
  std::string single_can_data = timestamp + " " + key_world + " " + can_id_hex + " " + can_data_hex;
  recv_datas_vec_.emplace_back(single_can_data);
  return true;
}

std::string RecordCan::UnpacDataTheadFunc(const std::string& src_data) {
  if (src_data.empty()) {
    LOG(ERROR) << "some can data is empty,maybe have some bug";
  }

  std::string timestamp = src_data.substr(8, 8);
  std::string channel = src_data.substr(2, 1);
  std::string can_id = src_data.substr(4, 4);                            // hex
  std::string can_data = src_data.substr(16, src_data.size() - 16);  // hex

  auto ts = *((double*)timestamp.c_str());

  std::string timestamp_s, timestamp_ns;
  mviz_record::type_conversion::SplitFloat(ts, timestamp_s, timestamp_ns);
  timestamp = timestamp_s + " " + timestamp_ns;
  int channel_int = static_cast<int>(*channel.c_str()) + 1;  // net is 0-7 but can_link is 1-8

  std::string channel_str = std::to_string(channel_int);
  std::string can_link = "can" + channel_str;  // can_mapping_table
  std::string key_world = "bm.0.can" + channel_str + "." + link_mapping_table_.at(can_link);
  std::string can_id_hex = mviz_record::type_conversion::int_to_hex(*(std::uint32_t*)(can_id.c_str()));
  std::string can_data_hex = mviz_record::type_conversion::string_to_hex(can_data);

  // std::cout << "[Info]:timestamp_s: " << timestamp_s << ",timestamp_ns:" << timestamp_ns << std::endl;
  // std::cout << "[Info]:key_world: " << key_world << std::endl;
  // std::cout << "[Info]:channnel: " << channel_int << std::endl;
  // std::cout << "[Info]:can_id: " << can_id_hex << std::endl;
  // std::cout << "[Info]:can_data: " << can_data_hex << std::endl;

  // mviz_can.txt: s、us、key_eorld、can-id(ox+hex)、can-data(hex)
  std::string unpack_data = timestamp + " " + key_world + " " + can_id_hex + " " + can_data_hex;

  return unpack_data;
}

bool RecordCan::CloseCanSocket() {
  int close_result = nn_shutdown(can_socket_, endpoint_id_);
  if (close_result < 0) {
    std::cerr << "[Error]:failed to close socket: " << nn_strerror(nn_errno()) << std::endl;
    return false;
  }
  nn_close(can_socket_);
  return true;
}

bool RecordCan::SaveCanData() {
  if (recv_datas_vec_.empty()) {
    std::cout << "[Info]:recv_datas_vec_ is empty" << std::endl;
    return false;
  }

  if (pCanRecordStream_ == nullptr) pCanRecordStream_ = new std::ofstream(data_path_, std::ios::out | std::ios::app);

  for (auto single_frame_can : recv_datas_vec_) {
    std::string unpack_can = UnpacDataTheadFunc(single_frame_can);
    (*pCanRecordStream_) << unpack_can << std::endl;
  }
  pCanRecordStream_->close();
  recv_datas_vec_.clear();  // clear have received data
  return true;
}

bool RecordCan::start(const std::string& currBatchIdDir) {
  bWantStart_ = true;
  bWantStop_ = false;
  data_path_ = currBatchIdDir + "/mviz_can.txt";

  std::cout << "[Info]:data_path_==" << data_path_ << std::endl;
  // 确保can socket 连接初始化连接成功
  if (pRecvCanDataThread_ != nullptr) return false;
  pRecvCanDataThread_ = new std::thread(&RecordCan::ReceiveDataThreadFunc, this);
  can_thread_id_ = pRecvCanDataThread_->get_id();
  std::cout << "[Info:] "
            << "can_thread_id_:[" << can_thread_id_ << " ] is running..." << std::endl;

  return true;
}
bool RecordCan::stop() {
  // 关掉线程和nomasg
  bWantStop_ = true;
  bWantStart_ = false;
  if (pRecvCanDataThread_ != nullptr) {
    if (pRecvCanDataThread_->joinable()) {
      pRecvCanDataThread_->join();
    }
    delete pRecvCanDataThread_;
    pRecvCanDataThread_ = nullptr;
  }

  if (pCanRecordStream_ != nullptr) {
    // stop ：save can_datas to loacl
    SaveCanData();
    if (pCanRecordStream_->is_open()) {
      pCanRecordStream_->close();
    }
    delete pCanRecordStream_;
    pCanRecordStream_ = nullptr;
  }

  return true;
}
void RecordCan::close() {
  if (CloseCanSocket()) {
    std::cout << "[Info]:can nanomsg close socket success" << std::endl;
  }
}
RecordCan::~RecordCan() {
  close();

  if (pRecvCanDataThread_ != nullptr) {
    if (pRecvCanDataThread_->joinable()) {
      pRecvCanDataThread_->join();
    }
    delete pRecvCanDataThread_;
    pRecvCanDataThread_ = nullptr;
  }

  if (pCanRecordStream_ != nullptr) {
    if (pCanRecordStream_->is_open()) {
      pCanRecordStream_->close();
    }
    delete pCanRecordStream_;
    pCanRecordStream_ = nullptr;
  }

  if (!recv_datas_vec_.empty()) recv_datas_vec_.clear();
}
}  // namespace collect_can
