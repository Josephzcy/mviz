//
// Created by nick on 23-3-30.
//

#ifndef MVIZ_RECORD_FILESYSRECORD_HPP_
#define MVIZ_RECORD_FILESYSRECORD_HPP_
#include <iomanip>
#include <sstream>
#include <thread>

#include "collect/collect_dbm.hpp"
#include "collect/file_sys.hpp"
#include "collect/topic_handle.hpp"


namespace minieye::mviz::file {
class CFileSysRecord : public CFileSys {
 public:
  CFileSysRecord(const char* saveRootDir, const char* topicName,const std::string& tick_unit) : CFileSys(saveRootDir) {
    _bWantStop = false;
    _bStart = false;
    _bWant = true;
    _p_thead = nullptr;
    dir_path = saveRootDir;
    topic_name = topicName;
    tick_unit_ = tick_unit;
    //    count = 0;
  }
  virtual ~CFileSysRecord() { _close(); }

  bool _start() {
    if (_p_thead != nullptr) return false;
    _p_thead = new std::thread(&CFileSysRecord::_thread_func, this);
    _threadId = _p_thead->get_id();
    LOG(INFO) << "file_save:[ " << topic_name << " ] - id:[ " << _threadId << " ] is running...";
    // return _bStart;
    return true;
  }

  void _close() {
    _bWantStop = true;
    if (_p_thead != nullptr) {
      _p_thead->join();
      delete _p_thead;
      _p_thead = nullptr;
      LOG(INFO) << "file_save: [ " << topic_name << "] -id: [ " << _threadId << " ] was exited...";
    }
    _bStart = false;
    file_sys_finalize();
  }

  inline void logRecvTheData(std::string topicName, size_t no, size_t size, const char* data) {
    std::stringstream ss;
    for (size_t i = 0; i < size; i++) {
      ss << std::hex << std::setw(2) << std::setfill('0') << size_t(*(data + i));
    }
    LOG(INFO) << "LibflowClient: [" << topicName << "] [" << no << "] size:" << size << " data :" << ss.str();
  }

  int write_bin_data(const char* buff, size_t n) override {
    binFileOFS.write(buff, n);
    return 0;
  };

  int write_bin_data(std::string buff) {
    binFileOFS.write(buff.c_str(), buff.size());
    return 0;
  };

  int write_img_bin_data(const char* dataC) {
    int32_t imgDataSize = *(reinterpret_cast<const int32_t*>(dataC + 20));
    binFileOFS.write(dataC + 36, imgDataSize);
    return 0;
  }

  int write_data_json(bool isImg, int32_t height = 0, int32_t width = 0) override {

    std::string _topic_name=("camera30" == topic_name ? "camera_stitching" : topic_name);
    std::string bin_str = _topic_name + ".bin";
    std::string index_json_str = _topic_name + ".index.json";

    if (isImg) {
      dataJson["device"] = "tda4";
      dataJson["protocol"] = "H.264";
      dataJson["data"].push_back(bin_str.c_str());
      dataJson["index"].push_back(index_json_str.c_str());
      dataJson["height"] = height;  // TODO
      dataJson["width"] = width;
    } else {
      dataJson["device"] = "tda4";
      dataJson["protocol"] = topic_name + ".proto";
      dataJson["data"].push_back(bin_str.c_str());
      dataJson["index"].push_back(index_json_str.c_str());
    }

    dataJsonFileOFS << std::setw(4) << dataJson << std::endl;
    // save_to_json(dataJson, dataJsonPath.string().c_str());
    dataJsonFileOFS.close();
    return 0;
  };

  int write_index_json(const nlohmann::json& one_frame_info) override {
    //      std::cout << "one_frame_info: " << one_frame_info << std::endl;
    //      std::cout << "one_frame_info size: " << one_frame_info.size() <<
    //      std::endl;
    totalIndexJson["index"].push_back(one_frame_info);
    //      LOG(INFO) << "totalIndexJson: " << totalIndexJson["index"] <<
    //      std::endl;
    // save_to_json(indexJson, indexJsonPath.string().c_str());
    return 0;
  };

  void save_to_json(const nlohmann::json& minieye_obj, const std::string& dst_dir,
                    const std::string& file_name) override {
    std::string target_file = dst_dir + "/" + file_name;
    std::cout << "saving target_file: " << target_file << std::endl;
    std::ofstream dest_file(target_file, std::ios::out);
    dest_file << std::setw(4) << minieye_obj << std::endl;
  };

 private:
  int _initialize() { return file_sys_initialize(topic_name); }

  void _thread_func() {
    _bStart = true;
    _bWantStop = false;
    LOG(INFO) << "file_save:[ " << topic_name << " ] starting...";
    if (_initialize() == 0) {
      LOG(INFO) << "file_save:[ " << topic_name << " ] initialize OK...";
    } else {
      LOG(INFO) << "file_save:[ " << topic_name << " ] initialize FAIL...";
    }



    topic::LibFlowRecvTopicHandler<std::string> topicRecvHandler(topic_name,tick_unit_);
    collect::Recv_Topic_Data_ST recvTopicDataSt;
    bool isImg = topicRecvHandler.IsImgTopic(topic_name);
    if (!isImg)
      init_write_index_json_head(totalIndexJson);
    else
      init_write_img_index_json_head(totalIndexJson);

    int count = 0, countImg = 0;
    while (!_bWantStop) {
      // 读取相关缓冲区数据进行数据写入
      if (collect::CDataCollectDBMan::get_instance()->get_topic_data(topic_name, recvTopicDataSt)) {
        // LogRecvTheData(topic_name,count,recvTopicDataSt.size,recvTopicDataSt.buff);
        //  可以发送事件，也可以定期查询QUEUE
        if (isImg) {
          countImg++;
          auto topic_msg = topicRecvHandler.GetImgBinData(recvTopicDataSt.buff, recvTopicDataSt.size);
          write_img_bin_data(recvTopicDataSt.buff);
          write_index_json(topicRecvHandler.GetOneImgInfo(topic_msg));
          if (1 == countImg) write_data_json(isImg, topic_msg.Height, topic_msg.Width);
        } else {
          count++;
          auto topic_msg = topicRecvHandler.GetBinData(recvTopicDataSt.buff, recvTopicDataSt.size);
          // write_bin_data(recvTopicDataSt.buff, recvTopicDataSt.size);
          write_bin_data(topicRecvHandler.str_road);
          write_index_json(topicRecvHandler.GetOneFrameInfo());
          if (1 == count) write_data_json(isImg);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if ((0 == countImg) && (0 == count)) {
      write_data_json(isImg);
    }

    indexJsonFileOFS << std::setw(4) << totalIndexJson << std::endl;
    indexJsonFileOFS.close();

    //      else { // camera30
    //        topic::LibFlowRecvTopicHandler<std::string>
    //        topicRecvHandler(topic_name); collect::Recv_Topic_Data_ST
    //        recvTopicDataSt; init_write_img_index_json_head(totalIndexJson);
    //        write_data_json(topic_name);
    //        while(!_bWantStop) {
    //          if(collect::CDataCollectDBMan::get_instance()->get_topic_data(topic_name,
    //          recvTopicDataSt)) {
    //            auto img_msg =
    //            topicRecvHandler.GetImgBinData(recvTopicDataSt.buff,
    //            recvTopicDataSt.size);
    //            write_img_bin_data(recvTopicDataSt.buff);
    //            write_index_json(topicRecvHandler.GetOneImgInfo(img_msg));
    //          }
    //          std::this_thread::sleep_for(std::chrono::milliseconds(5));
    //        }
    //        indexJsonFileOFS << std::setw(4) << totalIndexJson << std::endl;
    //        indexJsonFileOFS.close();
    //      }

    LOG(INFO) << "file_save:[ " << topic_name << " ] close....";
  }

  int count;
  bool _bWantStop;
  bool _bStart;
  bool _bWant;
  std::thread* _p_thead;
  std::thread::id _threadId;
  std::string dir_path;
  std::string topic_name;
  std::string tick_unit_{"us"};
  nlohmann::json dataJson;
  nlohmann::json totalIndexJson;
};
}  // namespace minieye::mviz::file
#endif  // MVIZ_RECORD_FILESYSRECORD_HPP_
