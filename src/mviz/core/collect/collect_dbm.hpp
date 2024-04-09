
#ifndef MVIZ_COLLECT_DBM_HPP
#define MVIZ_COLLECT_DBM_HPP

#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "concurrentqueue.h"

namespace minieye::mviz::collect {
typedef struct RECV_TOPIC_DATA_ST_ {
  char *buff;
  size_t size;
  RECV_TOPIC_DATA_ST_() {
    buff = nullptr;
    size = 0;
  };
} Recv_Topic_Data_ST;

typedef moodycamel::ConcurrentQueue<Recv_Topic_Data_ST> ConQueue;
typedef std::map<std::string, ConQueue *> RecvQueueMap;
static const char *viewer_name = "_viewer";

class CDataCollectDBMan {
 public:
  static CDataCollectDBMan *get_instance() { return pSingletonCollectDBMan; }

  static void delete_instance() {
    if (pSingletonCollectDBMan) {
      pSingletonCollectDBMan->release_queue_map(true);
      delete pSingletonCollectDBMan;
      pSingletonCollectDBMan = nullptr;
    }
  }

  bool init_to_dbm(std::vector<std::string> &topicVec) {
    if (bCanOperator) {
      for (const auto &elem : topicVec) {
        auto pValue = new moodycamel::ConcurrentQueue<Recv_Topic_Data_ST>(100);
        recvQueueMap.insert(std::make_pair(elem, pValue));
        auto pValue_viewer = new moodycamel::ConcurrentQueue<Recv_Topic_Data_ST>(100);
        recvQueueMap.insert(std::make_pair(elem + viewer_name, pValue_viewer));
      }
      return true;
    }
    return false;
  }

  bool insert_data(const std::string &topic, const char *data, const size_t size) {
    if (bCanOperator) {
      Recv_Topic_Data_ST recvTopicDataSt;        //
      Recv_Topic_Data_ST recvTopicViewerDataSt;  //
      recvTopicViewerDataSt.size = recvTopicDataSt.size = size;
      recvTopicDataSt.buff = new char[size];
      recvTopicViewerDataSt.buff = new char[size];
      memcpy(recvTopicDataSt.buff, data, size);  //
      memcpy(recvTopicViewerDataSt.buff, data, size);
      recvQueueMap[topic]->enqueue(recvTopicDataSt);  //
      recvQueueMap[topic + viewer_name]->enqueue(recvTopicViewerDataSt);
      return true;
    }
    return false;
  }

  bool get_topic_data(const std::string &topic, Recv_Topic_Data_ST &outData) {
    if (bCanOperator) {
      // @NOTICE 检查是否有key
      auto it = recvQueueMap.find(topic);
      if (it == recvQueueMap.end()) {
        return false;
      }
      if (recvQueueMap[topic]->size_approx() > 0) {
        Recv_Topic_Data_ST recvTopicDataSt;
        recvQueueMap[topic]->try_dequeue(recvTopicDataSt);
        outData.size = recvTopicDataSt.size;
        outData.buff = new char[outData.size];
        memcpy(outData.buff, recvTopicDataSt.buff, outData.size);
        delete recvTopicDataSt.buff;
        recvTopicDataSt.buff = nullptr;
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

  void drop_data() {
    if (bCanOperator) release_queue_map(false);
  }

 private:
  void clear_queue(ConQueue *pQueue) {
    bCanOperator = false;
    if (pQueue == nullptr) return;
    while (true) {
      auto bReadQueue = false;
      Recv_Topic_Data_ST dataSt;
      bReadQueue = pQueue->try_dequeue(dataSt);
      if (!bReadQueue) break;
      if (dataSt.buff != nullptr) {
        delete dataSt.buff;
        dataSt.buff = nullptr;
      }
    }
  }

  void release_queue_map(bool cmd) {
    bCanOperator = false;
    for (auto &it : recvQueueMap) {
      ConQueue *pQueue = it.second;
      if (pQueue == nullptr) continue;
      clear_queue(pQueue);
      if (cmd) {
        delete pQueue;
        pQueue = nullptr;
      }
    }
    if (!cmd) bCanOperator = true;
  }

  bool bCanOperator;

  CDataCollectDBMan() noexcept : bCanOperator(true){};
  RecvQueueMap recvQueueMap;
  static CDataCollectDBMan *pSingletonCollectDBMan;
};
}  // namespace minieye::mviz::collect

#endif  // MVIZ_COLLECT_DBM_HPP
