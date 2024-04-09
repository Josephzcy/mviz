#ifndef L2DATA_REDADER_HANDLE_H
#define L2DATA_REDADER_HANDLE_H
#include "global_value.h"
#include "l2_types.h"
#include "l2data_reader.h"

namespace mviz::replay {

using L2DataReaderPtr = std::shared_ptr<L2DataReader>;

class ReaderManager {
 private:
  ReaderManagerConfig m_reader_manager_config;
  std::vector<L2DataReaderPtr> readers;
  // L2DataReaderPtr key_topic_reader = nullptr;
  std::string key_topic;

 public:
  ReaderManager(/* args */);
  ~ReaderManager();
  // L2DataFramePtrList：一帧同步好的数据(包含各个信号，最多一帧)
  bool InitReaders();
  L2DataFramePtrList GetFrames(int frame_index);
  int GetKeyTopicDataLen();
  int key_topic_len{-1};
  bool is_available{true};
  L2DataReader *GetMainTopic();
};

}  // namespace mviz::replay

#endif