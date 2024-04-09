
//
// Created by nick on 23-4-4.
//
#include "topic_factory.h"

#include <sys/time.h>

#include <algorithm>
#include <ctime>
#include <functional>
#include <map>
#include <string>
// 3rdparty
#include <iostream>

namespace minieye::mviz::topic {

ClassFactory::~ClassFactory() {
  if (m_classMap.empty()) return;
  if (m_classMap.empty()) {
    for (auto &[key, pb_object] : m_classMap) {
      if (pb_object != nullptr) pb_object = nullptr;
    }  // std::varaint * set nullptr and memory will be released in ReleaseAction
  }

}  // namespace minieye::mviz::topic

PbType *ClassFactory::getClassByName(const std::string &topicName) {
  std::map<std::string, PbType *>::const_iterator iter = m_classMap.find(topicName);
  if (iter == m_classMap.end()) {
    std::cout << "===get topicName:" << topicName << " is nullptr" << std::endl;
    return nullptr;
  } else {
    return iter->second;
  }
}

void ClassFactory::registClass(const std::string &topicName, PbType *pPbObejctType) {
  m_classMap.insert(std::pair<std::string, PbType *>(topicName, pPbObejctType));
}

// 获取工厂类的单个实例对象
ClassFactory &ClassFactory::getInstance() {
  static ClassFactory instance;
  return instance;
}

std::map<std::string, PbType *> &ClassFactory::get_map() { return m_classMap; }

}  // namespace minieye::mviz::topic
