#include "reader_manager.h"
namespace mviz::replay {

ReaderManager::ReaderManager(/* args */) {
  is_available = InitReaders();
  // qDebug() <<"Func:"<< __FUNCTION__ << ",line:" << __LINE__;
}

bool ReaderManager::InitReaders() {
  int available_reader_num = 0;
  auto m_reader_manager_config = g_mrcfg->reader_manager_config;
  key_topic = m_reader_manager_config.key_topic;
  L2DataReader *key_topic_reader = nullptr;
  for (auto const &i : m_reader_manager_config.topic_configs) {
    auto reader_ptr = std::make_shared<L2DataReader>(m_reader_manager_config, i);
    if (reader_ptr->is_available) {
      readers.push_back(reader_ptr);
      available_reader_num++;
    }
    if (i.topic == m_reader_manager_config.key_topic) {
      key_topic_reader = reader_ptr.get();
    }
  }
  if (available_reader_num == 0) {
    qDebug() << "[Error] all reader is not available, check config!";
    return false;
  }
  if (key_topic_reader) {
    if (key_topic_reader->is_available) {
      key_topic_len = key_topic_reader->GetDataLen();
    } else {
      qDebug() << "[Error] key topic reader is not available, check config!";
      return false;
      // exit(1);
    }
  } else {
    qDebug() << "[Error] key topic reader is nullptr, check config!";
    return false;
    // exit(1);
  }
  for (const auto &reader : readers) {
    reader->MapToTopicMountainClimbing(key_topic_reader);
    // reader->MapToTopicMountainClimbing(key_topic_reader.get());
  }
  return true;
}

L2DataFramePtrList ReaderManager::GetFrames(int frame_index) {
  L2DataFramePtrList frame_list;
  for (const auto &reader : readers) {
    auto frame = reader->GetFrameFromIndex(frame_index);
    if (frame) {
      frame_list.push_back(frame);
    }
  }
  return frame_list;
}

int ReaderManager::GetKeyTopicDataLen() { return key_topic_len; }

L2DataReader *ReaderManager::GetMainTopic() {
  for (auto it = readers.begin(); it != readers.end(); it++) {
    if ((*it)->topic == key_topic) {
      return (*it).get();
    }
  }
  return nullptr;
}

ReaderManager::~ReaderManager() {}

}  // namespace mviz::replay