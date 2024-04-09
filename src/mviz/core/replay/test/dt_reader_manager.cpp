#include <iostream>
#include <memory>

#include "configuration_manager.h"
#include "global_value.h"
#include "reader_manager.h"
using namespace std;
using namespace mviz::replay;
using ReaderManagerPtr = std::shared_ptr<mviz::replay::ReaderManager>;

ReaderManagerPtr m_data_readmanager = nullptr;

mviz::replay::ReaderManager *ptr = nullptr;

void test1()  //单路视频偶有发生不能析构问题
{
  for (size_t i1 = 0; i1 < 10; i1++) {
    m_data_readmanager = std::make_shared<mviz::replay::ReaderManager>();
    for (size_t i = 0; i < 30; i++) {
      auto frames = m_data_readmanager->GetFrames(i);
      cout << "frame seq:" << i << std::endl;
    }

    cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
    if (m_data_readmanager != nullptr) {
      m_data_readmanager.reset();  // @TODO FIX这里会卡死界面，暂时无解，回放另一个数据时需要重启
    }
    m_data_readmanager = nullptr;
    cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
    m_data_readmanager = std::make_shared<mviz::replay::ReaderManager>();
    cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
    for (size_t i = 50; i < 100; i++) {
      auto frames = m_data_readmanager->GetFrames(i);
      cout << "frame seq:" << i << std::endl;
    }
    m_data_readmanager.reset();
  }
}
void test2() {
  ptr = new mviz::replay::ReaderManager;
  // for (size_t i = 0; i < 10; i++)
  // {
  //     auto frames = ptr->GetFrames(i);
  //     cout << "frame seq:" << i << std::endl;
  // }
  delete ptr;
  ptr = nullptr;

  // cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  // ptr = new mviz::replay::ReaderManager;
  // for (size_t i = 10; i < 20; i++)
  // {
  //     auto frames = ptr->GetFrames(i);
  //     cout << "frame seq:" << i << std::endl;
  // }
  // delete ptr;
  // ptr = nullptr;

  // cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  // auto data_readmanager = std::make_shared<mviz::replay::ReaderManager>();
  // cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  // for (size_t i = 20; i < 30; i++)
  // {
  //     auto frames = data_readmanager->GetFrames(i);
  //     cout << "frame seq:" << i << std::endl;
  // }
  // cout  << "reset:" <<std::endl;
  // m_data_readmanager.reset();
  // cout  << "reset" <<std::endl;
}
void test3() {
  mviz::replay::ReaderManager readmanager;
  for (size_t i = 0; i < 10; i++) {
    auto frames = readmanager.GetFrames(i);
    cout << "frame seq:" << i << std::endl;
  }
  cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  return;
}
void test4() {
  auto m_reader_manager_config = g_mrcfg->reader_manager_config;

  mviz::replay::L2DataReader reader(m_reader_manager_config, m_reader_manager_config.topic_configs[0]);
  reader.MapToTopicMountainClimbing(&reader);
  for (size_t i = 0; i < 10; i++) {
    auto frame = reader.GetFrameFromIndex(i);
    cout << "frame seq:" << i << std::endl;
  }
  cout << "----------------------------------" << std::endl;

  auto reader_ptr = new mviz::replay::L2DataReader(m_reader_manager_config, m_reader_manager_config.topic_configs[0]);
  reader_ptr->MapToTopicMountainClimbing(reader_ptr);
  for (size_t i = 10; i < 20; i++) {
    auto frame = reader_ptr->GetFrameFromIndex(i);
    cout << "frame seq:" << i << std::endl;
  }
  cout << "++++++++++++++++++++++++++" << std::endl;
  delete reader_ptr;
  return;
}

// 只实例化一个L2DataReader，reset再make_shared,正常析构
void test5() {
  auto m_reader_manager_config = g_mrcfg->reader_manager_config;

  auto reader_ptr =
      std::make_shared<mviz::replay::L2DataReader>(m_reader_manager_config, m_reader_manager_config.topic_configs[0]);
  reader_ptr->MapToTopicMountainClimbing(reader_ptr.get());
  for (size_t i = 10; i < 30; i++) {
    auto frame = reader_ptr->GetFrameFromIndex(i);
    cout << "frame seq:" << i << std::endl;
  }
  cout << "==================================" << std::endl;
  reader_ptr.reset();
  reader_ptr = nullptr;

  reader_ptr =
      std::make_shared<mviz::replay::L2DataReader>(m_reader_manager_config, m_reader_manager_config.topic_configs[0]);
  reader_ptr->MapToTopicMountainClimbing(reader_ptr.get());
  for (size_t i = 200; i < 250; i++) {
    auto frame = reader_ptr->GetFrameFromIndex(i);
    cout << "frame seq:" << i << std::endl;
  }
  cout << "==================================" << std::endl;
}

// 实例化2个L2DataReader，reset再make_shared,不正常析构
void test50() {
  auto m_reader_manager_config = g_mrcfg->reader_manager_config;

  auto reader_ptr =
      std::make_shared<mviz::replay::L2DataReader>(m_reader_manager_config, m_reader_manager_config.topic_configs[0]);
  reader_ptr->MapToTopicMountainClimbing(reader_ptr.get());
  for (size_t i = 10; i < 20; i++) {
    auto frame = reader_ptr->GetFrameFromIndex(i);
    cout << "frame seq:" << i << std::endl;
  }

  auto reader_ptr1 =
      std::make_shared<mviz::replay::L2DataReader>(m_reader_manager_config, m_reader_manager_config.topic_configs[0]);
  reader_ptr1->MapToTopicMountainClimbing(reader_ptr.get());
  for (size_t i = 100; i < 120; i++) {
    auto frame = reader_ptr1->GetFrameFromIndex(i);
    cout << "frame seq:" << i << std::endl;
  }

  cout << "==================================" << std::endl;
  // reader_ptr.reset();
  // reader_ptr = nullptr;
  // cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  // reader_ptr1.reset();
  // reader_ptr1 = nullptr;

  // reader_ptr = std::make_shared<mviz::replay::L2DataReader>(m_reader_manager_config,
  // m_reader_manager_config.topic_configs[0]); reader_ptr->MapToTopicMountainClimbing(reader_ptr.get()); for (size_t i
  // = 10; i < 20; i++)
  // {
  //     auto frame = reader_ptr->GetFrameFromIndex(i);
  //     cout << "frame seq:" << i << std::endl;
  // }
  // cout << "==================================" << std::endl;
}

// 实例化2个L2DataReader，直接裸指针,不正常析构
void test51() {
  auto m_reader_manager_config = g_mrcfg->reader_manager_config;

  auto reader_ptr = new mviz::replay::L2DataReader(m_reader_manager_config, m_reader_manager_config.topic_configs[0]);
  reader_ptr->MapToTopicMountainClimbing(reader_ptr);
  for (size_t i = 10; i < 20; i++) {
    auto frame = reader_ptr->GetFrameFromIndex(i);
    cout << "frame seq:" << i << std::endl;
  }

  // auto reader_ptr1 = new mviz::replay::L2DataReader(m_reader_manager_config,
  // m_reader_manager_config.topic_configs[0]); reader_ptr1->MapToTopicMountainClimbing(reader_ptr); for (size_t i =
  // 100; i < 120; i++)
  // {
  //     auto frame = reader_ptr1->GetFrameFromIndex(i);
  //     cout << "frame seq:" << i << std::endl;
  // }

  cout << "==================================" << std::endl;
  delete reader_ptr;
  reader_ptr = nullptr;
  cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  // delete reader_ptr1;
  // reader_ptr1 = nullptr;
}

// 测试VideoBinReadDecode
//
bool test60() {
  VideoBinReadDecode *video_reader = nullptr;
  VideoBinReadDecode *video_reader1 = nullptr;
  auto manager_config = g_mrcfg->reader_manager_config;
  nlohmann::json data_json_item;
  nlohmann::json index_json_item;
  auto m_topic_config = manager_config.topic_configs[0];
  auto m_mviz_data_path = manager_config.mviz_data_path;

  std::string dataJsonPath = pystring::os::path::join(m_mviz_data_path, m_topic_config.data_json_path);
  std::ifstream dataJsonStream(dataJsonPath, std::ifstream::in);
  if (!dataJsonStream.is_open()) {
    std::cout << "[ERROR] file open fail: " << dataJsonPath << std::endl;
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
    std::cout << "[ERROR] file open fail: " << indexJsonPath << std::endl;
    return false;
  }
  indexJsonStream >> index_json_item;

  std::string binPath = pystring::os::path::join(m_mviz_data_path, data_json_item["data"][0].get<std::string>());

  video_reader = new VideoBinReadDecode(data_json_item, index_json_item, binPath, "H264");
  for (size_t i = 0; i < 10; i++) {
    std::shared_ptr<cv::Mat> result = std::make_shared<cv::Mat>();
    int count = 0;
    while (true) {
      int ret = (video_reader->GetMat(*result, i));
      if (ret == 0) {
        cout << "0frame seq:" << i << std::endl;
        break;
      }

      if (count > 30) {
        std::cout << "Get video fail, decode time out\n";
        break;
      }
      usleep(100);
      count++;
    }
  }
  cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;

  video_reader1 = new VideoBinReadDecode(data_json_item, index_json_item, binPath, "H264");
  for (size_t i = 0; i < 10; i++) {
    std::shared_ptr<cv::Mat> result = std::make_shared<cv::Mat>();
    int count = 0;
    while (true) {
      int ret = (video_reader1->GetMat(*result, i));
      if (ret == 0) {
        cout << "1frame seq:" << i << std::endl;
        break;
      }

      if (count > 30) {
        std::cout << "Get video fail, decode time out\n";
        break;
      }
      usleep(100);
      count++;
    }
  }

  cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  delete video_reader;
  video_reader = nullptr;
  cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  delete video_reader1;
  video_reader1 = nullptr;

  // cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  // return true;
  // video_reader = new VideoBinReadDecode(data_json_item, index_json_item,
  //                                         binPath, "H264");
  // video_reader1 = new VideoBinReadDecode(data_json_item, index_json_item,
  //                                         binPath, "H264");
  return true;
}

void test61() {}

int main() {
  std::string path = "/root/mviz2_ros/src/mviz/config/mviz_replay.yaml";
  mviz::replay::ConfigurationManager config_manager(path);
  g_mrcfg.reset(new mviz::replay::MvizReplayConfig);
  *g_mrcfg = config_manager.GetConfig();
  int a = 1;
  cin >> a;
  if (a == 1) {
    test1();
  }
  if (a == 2) {
    test2();
  }
  if (a == 3) {
    test3();
  }
  if (a == 4) {
    test4();
  }
  if (a == 5) {
    test5();
  }
  if (a == 50) {
    test50();
  }
  if (a == 51) {
    test51();
  }
  if (a == 60) {
    test60();
  }
  if (a == 61) {
    test61();
  }

  cout << __FUNCTION__ << ":::::::::::::::::::::::::::::::::::line " << __LINE__ << std::endl;
  return 0;
}

/*
clear && rosrun  mviz dt_test
clear && rosrun --prefix 'gdb -ex run --args'  mviz dt_test
*/