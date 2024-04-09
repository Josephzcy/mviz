//
// Created by nick on 23-4-4.
//

#ifndef MVIZ_TOPIC_FACTORY_HPP_
#define MVIZ_TOPIC_FACTORY_HPP_
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
#include <variant>

#include "glog/logging.h"
#include "google/protobuf/message.h"
#include "json.hpp"
#include "msgpack.hpp"

#include "platform_defination.h"
namespace minieye::mviz::topic {
class ClassFactory {
 private:
  std::map<std::string, PbType *> m_classMap;
  ClassFactory() { LOG(INFO) << "ClassFactory: is construct" << std::endl; };
  ClassFactory(const ClassFactory &) = delete;
  ClassFactory operator=(const ClassFactory &) = delete;

 public:
  ~ClassFactory();

 public:
  std::map<std::string, PbType *> &get_map();

  PbType *getClassByName(const std::string &topicName);

  void registClass(const std::string &topicName, PbType *pPbObejctType);

  // 获取工厂类的单个实例对象
  static ClassFactory &getInstance();
};

}  // namespace minieye::mviz::topic

#endif  // MVIZ_TOPIC_HANDLE_HPP_
