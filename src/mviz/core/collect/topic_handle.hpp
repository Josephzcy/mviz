//
// Created by nick on 23-4-4.
//

#ifndef MVIZ_TOPIC_HANDLE_HPP_
#define MVIZ_TOPIC_HANDLE_HPP_
#include <sys/time.h>

#include <algorithm>
#include <ctime>
#include <functional>
#include <map>
#include <string>
#include <typeinfo>
#include <vector>
// 3rdparty
#include <iostream>

#include "msgpack.hpp"
#include "topic_factory.h"

namespace minieye::mviz::topic {
struct NewHilMsg {
  std::string topic;
  uint32_t frame_id;
  uint32_t time_s;
  uint32_t time_us;
  std::vector<char> data;
  MSGPACK_DEFINE_MAP(topic, frame_id, time_s, time_us, data);
};

struct ImageFlowHeaderStruct {
  int32_t Height;
  int32_t Width;
  uint32_t SendTimeHigh;
  uint32_t SendTimeLow;
  int32_t FrameType;
  int32_t DataSize;
  uint32_t Seq;
  uint32_t Sec;
  uint32_t Nsec;
};

// ===== 2 ============
#if 0
// 下面的方法可以动态，上面是静态
Test1 *ObjectCreator_1(){
  return new Test1;
}
RegisterAction CreatorRegister_1("Test1", (PTRCreateObject) ObjectCreator_1);
Test2 *ObjectCreator_2(){
  return new Test2;
}
RegisterAction CreatorRegister_2("Test2", (PTRCreateObject) ObjectCreator_2);
#endif

#if 1
// 通过可变PB可变参数获取相关的处理结果
template <typename TN>
class TopicManagerBase {
 public:
  explicit TopicManagerBase(TN &topicName, const std::string &tick_unit)
      : _topicName(topicName), tick_unit_(tick_unit) {}
  virtual ~TopicManagerBase() = default;
  virtual void Init() {}
  virtual const NewHilMsg &GetBinData(const char *dataC, const size_t size) {
    DeserialRecvPbData(dataC, size);
    return srcMsg;
  }

  const nlohmann::json GetOneFrameInfo() { return RecvPBToJsonData(); };

  const nlohmann::json GetOneImgInfo(const ImageFlowHeaderStruct &img_msg) { return RecvImgToJsonData(img_msg); };

  const ImageFlowHeaderStruct &GetImgBinData(const char *dataC, const size_t size) {
    // 解析帧头信息
    int32_t imgHeight = *(reinterpret_cast<const int32_t *>(dataC));
    int32_t imgWidth = *(reinterpret_cast<const int32_t *>(dataC + 4));
    // @Notice：hil端SendTimeHigh(s)和Sec(s)填充了相同的内容
    // @Notice：hil端SendTimeLow(us)和Nsec(us)填充了相同的内容
    uint32_t imgSendTimeHigh = *(reinterpret_cast<const uint32_t *>(dataC + 8));
    uint32_t imgSendTimeLow = *(reinterpret_cast<const uint32_t *>(dataC + 12));
    int32_t frameType = *(reinterpret_cast<const int32_t *>(dataC + 16));
    int32_t imgDataSize = *(reinterpret_cast<const int32_t *>(dataC + 20));
    uint32_t imgSeq = *(reinterpret_cast<const uint32_t *>(dataC + 24));
    uint32_t imgSec = *(reinterpret_cast<const uint32_t *>(dataC + 28));
    uint32_t imgNsec = *(reinterpret_cast<const uint32_t *>(dataC + 32));
    // imgNsec *= 1000;  // 转成纳秒 ns

    header.Height = imgHeight;
    header.Width = imgWidth;
    header.SendTimeHigh = imgSendTimeHigh;
    header.SendTimeLow = imgSendTimeLow;
    header.Seq = imgSeq;
    header.DataSize = imgDataSize;
    header.Sec = imgSec;
    header.Nsec = imgNsec;
    header.FrameType = frameType;

    return header;
  }

  bool IsImg(const std::string &topic_name) {  // 判断是否是图像数据
    return IsImgTopic(topic_name);
  }

  bool IsImgTopic(const std::string &topic_name) {  // 判断是否是图像数据
    std::vector<std::string> list = {"camera", "video", "bev"};
    std::string str = topic_name;
    transform(str.begin(), str.end(), str.begin(), ::tolower);

    int n = list.size();
    for (int i = 0; i < n; ++i) {
      if (str.find(list[i].c_str()) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  uint64_t get_timestamp() const { return timestamp; }

  uint64_t get_tick() const { return tick; }

 private:
  inline void logRecvTheData(const char *topicName, int no, size_t size, const char *data) {
    std::stringstream ss;
    for (int i = 0; i < size; i++) {
      ss << std::hex << std::setw(2) << std::setfill('0') << int(*(data + i));
    }
    LOG(INFO) << "LibflowClient: [" << topicName << "] [" << no << "] size:" << size << " data :" << ss.str();
  }

  void DeserialRecvPbData(const char *dataC, const size_t size) {
    //    logRecvTheData(_topicName.c_str(), count, size, dataC);

    auto handle = msgpack::unpack(dataC, size);  // 输入二进制数据
    auto obj = handle.get();                     // 得到反序列化对象
    obj.convert(srcMsg);
    str_road.clear();
    str_road.insert(str_road.begin(), srcMsg.data.begin(),
                    srcMsg.data.end());  // protobuf 反序列化
    PerTopicTypeRecvHandle();
    //    LOG(INFO) << "recv " << _topicName << ",size: " << srcMsg.data.size();
  }

  const nlohmann::json RecvPBToJsonData() {
    nlohmann::json one_frame_info;
    one_frame_info.emplace_back(std::to_string(timestamp));
    one_frame_info.emplace_back(offset);
    one_frame_info.emplace_back(srcMsg.data.size());
    one_frame_info.emplace_back(count++);
    one_frame_info.emplace_back(std::to_string(tick));
    offset += srcMsg.data.size();
    return one_frame_info;
  }

  const nlohmann::json RecvImgToJsonData(const ImageFlowHeaderStruct &img_msg) {
    nlohmann::json one_fish_image_index;
    uint64_t _timestamp = static_cast<uint64_t>(img_msg.SendTimeHigh) * 1e6 + img_msg.SendTimeLow * 1e-3;
    uint64_t tick = static_cast<uint64_t>(img_msg.Sec) * 1e6 + img_msg.Nsec * 1e-3;
    LOG(INFO) << "High:" << img_msg.SendTimeHigh << " Low:" << img_msg.SendTimeLow << " Sec:" << img_msg.Sec
              << " Nsec:" << img_msg.Nsec;
    one_fish_image_index.emplace_back(std::to_string(_timestamp));
    one_fish_image_index.emplace_back(offset);
    one_fish_image_index.emplace_back(img_msg.DataSize);
    one_fish_image_index.emplace_back(img_msg.Seq);
    one_fish_image_index.emplace_back(count++);
    one_fish_image_index.emplace_back(0);
    one_fish_image_index.emplace_back(0);
    one_fish_image_index.emplace_back(img_msg.FrameType);
    one_fish_image_index.emplace_back(std::to_string(tick));
    offset += img_msg.DataSize;
    return one_fish_image_index;
  }

  // 修改为 绑定
  void PerTopicTypeRecvHandle() {
    PbType *topic_object = ClassFactory::getInstance().get_map()[_topicName];
    if (topic_object == nullptr) {
      LOG(INFO) << " topic: [ " << _topicName << " ] is not instantiated " << std::endl;
      return;
    }
    std::visit(
        [this](auto &&pb_message) {
          using TopicType = std::decay_t<decltype(pb_message)>;
          pb_message.ParseFromString(str_road);  // pb_msg基类
#ifdef MVIZ_TDA4
          if constexpr (std::is_same_v<TopicType, minieye::ImuDataList>) {  // 如果类型是int
            if (pb_message.imu_datas_size() != 0) {
              timestamp = pb_message.imu_datas(0).timestamp();  // timestamp = 0;
              tick = pb_message.imu_datas(0).tick();
              if (tick_unit_ == "ms") {
                tick = tick * 1e3;
              }
            } else if (_topicName == "imu_rtk") {
              if (pb_message.phy_imu_datas_size() != 0) {
                timestamp = pb_message.phy_imu_datas(0).timestamp();  // timestamp = 0;
                tick = pb_message.phy_imu_datas(0).tick();
                if (tick_unit_ == "ms") {
                  tick = tick * 1e3;
                }
              }
            }
          } else if constexpr (std::is_same_v<TopicType, minieye::APAStateControl>) {
            timestamp = pb_message.timestamp();
            tick = pb_message.tick();
            // uint32_t state = pb_message.state();
            if (state == 2 && pb_message.state() == 3) {
              start_times++;
              std::string timeStr = GetBeijingTimeStr();
              std::stringstream start_key;
              start_key << "startTime_hmi" << start_times;
              hmiTime_json[start_key.str()] = timeStr;
            }
            if (state == 3 && pb_message.state() == 7) {
              stop_times++;
              std::string timeStr = GetBeijingTimeStr();
              std::stringstream stop_key;
              stop_key << "stopTime_hmi" << stop_times;
              hmiTime_json[stop_key.str()] = timeStr;
            }
            state = pb_message.state();
          } else {
            timestamp = pb_message.timestamp();
            tick = pb_message.tick();
            if (tick_unit_ == "ms") {
              tick = tick * 1e3;
            }
          }
#endif
#ifdef MVIZ_j3
          // pb_message.ParseFromString(str_road);
          timestamp = pb_message.timestamp();
          tick = pb_message.tick();
          if (tick_unit_ == "ms") {
            tick = tick * 1e3;
          }
#endif
        },
        *topic_object);
    return;
  }

  std::string GetBeijingTimeStr() {
    // 获取当前时间
    std::time_t current_time = std::time(nullptr);

    // 将时间转换为本地时间格式
    std::tm *local_time = std::localtime(&current_time);
    // local_time->tm_hour = (local_time->tm_hour + 8) % 24;
    char myStr[25];
    std::string myFormat = "%Y-%m-%d %H:%M:%S";
    strftime(myStr, 25, myFormat.c_str(), local_time);
    for (int i = 0; myStr[i]; ++i) std::cout << myStr[i];
    return std::string(myStr);
  }

  std::string _topicName;
  NewHilMsg srcMsg;
  ImageFlowHeaderStruct header;
  uint64_t timestamp;
  uint64_t tick;
  uint64_t offset{0};
  int count{0};
  std::string tick_unit_{"us"};
  int state{-1};
  int start_times{0};
  int stop_times{0};

 public:
  std::string str_road;
  static nlohmann::json hmiTime_json;
};

template <typename TN>
class LibFlowRecvTopicHandler : public TopicManagerBase<TN> {
 public:
  using ParentClass = TopicManagerBase<TN>;
  explicit LibFlowRecvTopicHandler(TN topicName, const std::string &tick_unit) : ParentClass(topicName, tick_unit) {
    LOG(INFO) << " LibFlow Topic" << topicName << "Handler init ...";
  }
  // 可以在退出时 UNREGISTER
  virtual ~LibFlowRecvTopicHandler() = default;
  void Init() override {
    // 可以动态加载自己的REGISTER，也可以静态的加载自己的数据
  }
};
#endif
}  // namespace minieye::mviz::topic

#endif  // MVIZ_TOPIC_HANDLE_HPP_
