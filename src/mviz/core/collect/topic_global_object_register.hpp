
#ifndef TOPIC_GLOBAL_OBJECT_REGISTER_HPP_
#define TOPIC_GLOBAL_OBJECT_REGISTER_HPP_

#include "topic_factory.h"

namespace minieye::mviz::topic_register {

#define REGISTER(topicName, className)                                         \
  className object_##topicName;                                                \
  PbType *pDerived_##topicName = new PbType(object_##topicName);               \
  RegisterAction CreatorRegister##topicName(#topicName, pDerived_##topicName); \
  ReleaseAction ReleaseAction##topicName(#topicName, pDerived_##topicName);

class RegisterAction {
 public:
  RegisterAction(const std::string &topicName, PbType *pPbObejctType) {
    minieye::mviz::topic::ClassFactory::getInstance().registClass(topicName, pPbObejctType);
  }
};

class ReleaseAction {
 public:
  explicit ReleaseAction(const std::string topic, PbType *created_object) {
    topic_ = topic;
    created_object_ = created_object;
  }

  ~ReleaseAction() {
    LOG(INFO) << "gloabal object: [" << topic_ << " ] has released..." << std::endl;
    if (created_object_ != nullptr) {
      delete created_object_;
      created_object_ = nullptr;
    }
  }
  PbType *created_object_{nullptr};
  std::string topic_;
};

#ifdef MVIZ_TDA4
#include "tda4_vm/havp_vm_object_register.h"
// #include "tda4_vm/apa_vm_object_register.h"
#endif

#ifdef MVIZ_J3
#include "j3/object_register.h"
#endif

#endif

}  // namespace minieye::mviz::topic
