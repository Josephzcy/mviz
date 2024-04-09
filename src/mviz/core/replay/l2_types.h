#ifndef L2_TYPES_H
#define L2_TYPES_H
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "json.hpp"

namespace mviz::replay {

struct FrameInfo {
  int32_t self_index;
  int32_t g_index;
  int32_t size;
  int32_t offset;
  std::string time_stamp;
  std::string tick;
};

// 一个L2数据的单帧数据
struct L2DataFrame {
  FrameInfo info;
  std::string topic;
  std::vector<char> data;  //存序列化后的pb 与bgr二选一
  cv::Mat bgr;             //存图像 与data二选一
};

typedef std::map<std::string, L2DataFrame> L2DataFrameMap;
typedef std::shared_ptr<L2DataFrame> L2DataFramePtr;
typedef std::vector<L2DataFrame> L2DataFrameList;
typedef std::vector<L2DataFramePtr> L2DataFramePtrList;

// config from yaml
struct TopicConfig {
  std::string topic;
  std::string data_json_path;
  std::string sync_by;
  // std::string logsim_save_file;
};

struct ReaderManagerConfig {
  std::string mviz_data_path;
  std::string key_topic;
  int sync_dt_threshold_ms{100};
  std::string video_codecs_type;
  std::vector<TopicConfig> topic_configs;
};

/*
tda4泊车， 2：前； 4：后； 1：左； 3：右
1 | 3
2 | 4

tda4行车，1：前；2：后
1 | 2
*/
struct CameraSwitchingConfig {
  int image_front_position{2};
  int image_rear_position{4};
  int image_left_position{1};
  int image_right_position{3};
};

struct PubCameraConfig {
  std::string topic;
  int camera_number;
  std::vector<int> camera_position;
};

struct PubManagerConfig {
  std::string car_body_dir;
  float marker_radio{1.0};
  float marker_scale{0.02};
  std::vector<PubCameraConfig> pub_camera_config_list;
};

struct MvizReplayConfig {
  ReaderManagerConfig reader_manager_config;
  CameraSwitchingConfig camera_stitching_config;
  PubManagerConfig pub_manager_config;
};

}  // namespace mviz::replay

#endif