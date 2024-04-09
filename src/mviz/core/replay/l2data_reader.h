#ifndef L2DATA_READER_H
#define L2DATA_READER_H
#include <unistd.h>

#include <QDebug>
#include <fstream>
#include <iostream>
#include <map>

#include "l2_types.h"
#include "pystring.h"
#include "video_stream_decoder.h"

bool IsImgTopic(const std::string &topic_name);
namespace mviz::replay {

class L2DataReader {
 private:
  std::string m_mviz_data_path;
  TopicConfig m_topic_config;
  nlohmann::json data_json_item;
  nlohmann::json index_json_item;
  std::ifstream bin_stream;
  nlohmann::json fields_map{
      {"timestamp", 0}, {"offset", 1}, {"size", 2}, {"frame_idx", 3}, {"tick", 4},
  };
  std::map<std::string, int64> unit_map = {
      {"s", 1000000},
      {"ms", 1000},
      {"us", 1},
  };
  nlohmann::json index_list;

  int data_len{-1};
  VideoBinReadDecode *video_reader = nullptr;
  std::string video_codecs_type{"H264"};

  int m_sync_dt_threshold_us;
  std::string self_field, main_field;
  std::string self_field_unit, main_field_unit;
  std::map<int, int> mapindex_to_selfindex;  // key_topic 到当前 topic的 映射

 private:
  bool LoadDataJson();
  int64 get_timestamp_json(nlohmann::json &data, std::string unit = "us");

 public:
  L2DataReader(const std::string mviz_data_path, const TopicConfig &config, std::string, int threshold_ms = 150);
  L2DataReader(const ReaderManagerConfig manager_config, const TopicConfig &config);
  ~L2DataReader();

  bool is_available{false};
  bool is_image{false};
  std::string topic;
  void MapToTopicMountainClimbing(L2DataReader *key_topic_reader);
  std::string GetTopicName();
  int GetDataLen();
  L2DataFramePtr GetFrameFromIndex(int index);
};

}  // namespace mviz::replay
#endif
