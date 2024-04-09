#include "l2data_reader.h"

#include "video_stream_decoder.h"

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

namespace mviz::replay {
L2DataReader::L2DataReader(const std::string mviz_data_path, const TopicConfig &config, std::string codecs_type,
                           int threshold_ms)
    : m_mviz_data_path(mviz_data_path),
      m_topic_config(config),
      video_codecs_type(codecs_type),
      m_sync_dt_threshold_us(threshold_ms * 1000) {
  qDebug() << "[INFO] load topic:" << QString::fromStdString(m_topic_config.topic);
  is_image = IsImgTopic(config.topic);
  is_available = LoadDataJson();
  self_field_unit = "us";
  main_field_unit = "us";
  self_field = config.sync_by;
  main_field = config.sync_by;
  topic = config.topic;
}

L2DataReader::L2DataReader(const ReaderManagerConfig manager_config, const TopicConfig &config)
    : L2DataReader(manager_config.mviz_data_path, config, manager_config.video_codecs_type,
                   manager_config.sync_dt_threshold_ms) {}

bool L2DataReader::LoadDataJson() {
  std::string dataJsonPath = pystring::os::path::join(m_mviz_data_path, m_topic_config.data_json_path);
  std::ifstream dataJsonStream(dataJsonPath, std::ifstream::in);
  if (!dataJsonStream.is_open()) {
    // std::cout << "[ERROR] file open fail: "  << dataJsonPath << std::endl;
    qDebug() << "[ERROR] file open fail: " << dataJsonPath.c_str();
    return false;
  }
  dataJsonStream >> data_json_item;
  std::cout << data_json_item << std::endl;
  if (data_json_item["data"].size() < 1 || data_json_item["index"].size() < 1) {
    return false;
  }
  std::string indexJsonPath = pystring::os::path::join(m_mviz_data_path, data_json_item["index"][0].get<std::string>());
  std::ifstream indexJsonStream(indexJsonPath, std::ifstream::in);
  if (!indexJsonStream.is_open()) {
    // std::cout << "[ERROR] file open fail: " << indexJsonPath << std::endl ;
    qDebug() << "[ERROR] file open fail: " << indexJsonPath.c_str();
    return false;
  }
  indexJsonStream >> index_json_item;

  auto iter = index_json_item.find("fields");
  if (iter != index_json_item.end()) {
    fields_map.clear();
    fields_map = index_json_item["fields"];
  }

  auto iter1 = index_json_item.find("index");
  if (iter1 != index_json_item.end()) {
    data_len = index_json_item["index"].size();
    index_list = index_json_item["index"];
  } else {
    // std::cout << "[ERROR] index is null: "  << indexJsonPath << std::endl;
    qDebug() << "[ERROR] index is null: " << indexJsonPath.c_str();
    return false;
  }

  std::string binPath = pystring::os::path::join(m_mviz_data_path, data_json_item["data"][0].get<std::string>());
  bin_stream.open(binPath, std::ios::in | std::ios::binary);
  if (!bin_stream.is_open()) {
    // std::cout << "[ERROR] file open fail: "  << binPath << std::endl;
    qDebug() << "[ERROR] file open fail: " << binPath.c_str();
    return false;
  }

  if (is_image) {
    // std::string format = "H265";
    video_reader = new VideoBinReadDecode(data_json_item, index_json_item, binPath, video_codecs_type);
  }
  return true;
}

int64 L2DataReader::get_timestamp_json(nlohmann::json &data, std::string unit) {
  int64 ans = 0;
  if (data.is_string()) {
    ans = std::atoll(data.get<std::string>().c_str());
  } else {
    ans = data.get<int64>();
  }
  // std::cout << ans << std::endl;
  return ans * unit_map[unit];
}

void L2DataReader::MapToTopicMountainClimbing(L2DataReader *map_topic) {
  if (map_topic == nullptr) {
    return;
  }

  if (map_topic == this)
  // if(map_topic->m_topic_config.topic == this->m_topic_config.topic)
  {
    for (size_t i = 0; i < index_list.size(); i++) {
      mapindex_to_selfindex[i] = i;
    }
    return;
  }
  //    qDebug() << topic_name.c_str() ;
  int self_field_index = this->fields_map[self_field.c_str()];
  int map_field_index = map_topic->fields_map[main_field.c_str()];
  std::map<int, int> selfindex_to_mapindex;
  size_t self_flag = 0;
  size_t map_flag = 0;
  bool break_flag = false;

  while (map_flag + 1 < map_topic->index_list.size() && self_flag < index_list.size()) {
    // qDebug() << self_flag << " " << map_flag;
    // qDebug() << self_flag << " " << index_list.size();

    int64 self_field_data = get_timestamp_json(index_list[self_flag][self_field_index], self_field_unit);
    int64 map_field_data = get_timestamp_json(map_topic->index_list[map_flag][map_field_index], main_field_unit);
    int64 mapf_field_data = get_timestamp_json(map_topic->index_list[map_flag + 1][map_field_index], main_field_unit);

    while (llabs(self_field_data - map_field_data) > llabs(self_field_data - mapf_field_data)) {
      map_flag++;
      if (map_flag + 1 >= map_topic->index_list.size()) {
        break_flag = true;
        break;
      }

      map_field_data = get_timestamp_json(map_topic->index_list[map_flag][map_field_index], main_field_unit);
      mapf_field_data = get_timestamp_json(map_topic->index_list[map_flag + 1][map_field_index], main_field_unit);
    }

    if (break_flag) {
      break;
    }

    if (llabs(map_field_data - self_field_data) < m_sync_dt_threshold_us) {
      selfindex_to_mapindex[self_flag] = map_flag;
    }
    self_flag++;
  }

  for (auto const &pair : selfindex_to_mapindex) {
    mapindex_to_selfindex[pair.second] = pair.first;
  }
}

std::string L2DataReader::GetTopicName() { return m_topic_config.topic; }

int L2DataReader::GetDataLen() { return data_len; }

L2DataFramePtr L2DataReader::GetFrameFromIndex(int index) {
  if (!is_available) {
    return nullptr;
  }
  if (mapindex_to_selfindex.find(index) == mapindex_to_selfindex.end()) {
    return nullptr;
  }
  int self_index = mapindex_to_selfindex[index];

  auto buffer = std::make_shared<L2DataFrame>();
  buffer->topic = GetTopicName();
  buffer->info.g_index = index;
  buffer->info.self_index = self_index;

  nlohmann::json index_data = index_list[self_index];
  int offset_index = fields_map["offset"];
  int size_index = fields_map["size"];
  int offset = index_data[offset_index];
  int size = index_data[size_index];

  buffer->info.offset = offset;
  buffer->info.size = size;

  if (fields_map.find("tick") != fields_map.end()) {
    int tick_index = fields_map["tick"];
    buffer->info.tick = index_data[tick_index];
  }

  if (!is_image) {
    buffer->data.resize(size);
    bin_stream.seekg(offset, std::ios::beg);
    bin_stream.read(buffer->data.data(), size);
  } else {
    std::shared_ptr<cv::Mat> result = std::make_shared<cv::Mat>();
    int count = 0;
    while (true) {
      int ret = (video_reader->GetMat(*result, self_index));
      if (ret == 0) {
        break;
      }

      if (count > 30) {
        std::cout << "Get video fail, decode time out\n";
        break;
      }
      usleep(100);
      count++;
    }
    buffer->bgr = *result;
    // cv::imshow("bgr", buffer->bgr(cv::Rect(0,0,300,300)));
    // cv::waitKey(20);
  }
  // qDebug()
  //     << QString::fromStdString(buffer->topic)
  //     << ",tick:" << QString::fromStdString(buffer->info.tick)
  //     << ","<< buffer->info.offset;
  // qDebug()
  //     << QString::fromStdString(buffer->topic)
  //     << ",pointer address:" << static_cast<void*>(buffer.get());
  return buffer;
}

L2DataReader::~L2DataReader() {
  if (is_image) {
    delete video_reader;
    video_reader = nullptr;
  }

  if (bin_stream.is_open()) {
    bin_stream.close();
  }
  // qDebug()
  //     << QString::fromStdString(GetTopicName()) << ":"
  //     << __FUNCTION__ << ",line:" << __LINE__;
}
}  // namespace mviz::replay
