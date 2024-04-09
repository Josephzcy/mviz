#ifndef TEMPLATE_FACTORY_HPP_
#define TEMPLATE_FACTORY_HPP_

// 模板工厂 + function+ 返回智能指针封装成函数对象(lamada) + 单例模式(map) + 延迟加载 ;T for base class
#include <iostream>
#include <map>
#include <unordered_map>

#include "platform_defination.h"

// #include "j3_platform.hpp"
// #include "tda4_vm_platform.hpp"
namespace mviz::utils {
template <typename T>
class TemplateFactory final {
  using ObjectPtr = std::unique_ptr<T>;
  using CreateObjectPTR = std::function<ObjectPtr()>;
  using ObjectMap = std::map<std::string, CreateObjectPTR>;

 public:
  TemplateFactory(const TemplateFactory&) = delete;
  TemplateFactory& operator=(const TemplateFactory&) = delete;
  TemplateFactory(TemplateFactory&&) = delete;

 public:
  // 对外接口
  static void RegisterType(const std::string& type_name, CreateObjectPTR creator) {
    auto& instance_map = GetCreateInstanceMap();
    instance_map[type_name] = creator;
  }

  static ObjectPtr GetTypeInstance(const std::string& type_name) {
    auto& instance_map = GetCreateInstanceMap();
    auto it = instance_map.find(type_name);
    if (it != instance_map.end()) {
      return instance_map[type_name]();
    }
    return nullptr;
  }

 private:
  static ObjectMap& GetCreateInstanceMap() {
    static ObjectMap instance_map;
    return instance_map;
  }

  static T* GetInstance() {
    static T* instance = new T;
    return instance;
  };

 private:
  TemplateFactory() = default;

};  // classTemplateFactory



/*
static bool j3_platform = []() {
  TemplateFactory<mviz::platform::PlatformInterface>::RegisterType(
      "j3", []() { return std::make_unique<mviz::platform::J3Platform>(); });
  std::cout << "register tda4vm platform" << std::endl;

  return true;
}();
static bool tda4_platform = []() {
  TemplateFactory<mviz::platform::PlatformInterface>::RegisterType(
      "tda4", []() { return std::make_unique<mviz::platform::Tda4VmPlatform>(); });
  std::cout << "register tda4vm platform" << std::endl;
  return true;
}();


std::unique_ptr<mviz::platform::PlatformInterface> j3 =
    mviz::utils::TemplateFactory<mviz::platform::PlatformInterface>::GetTypeInstance("j3");
std::unique_ptr<mviz::platform::PlatformInterface> tda4 =
    mviz::utils::TemplateFactory<mviz::platform::PlatformInterface>::GetTypeInstance("tda4");
*/


/*
std::unique_ptr<mviz::platform::PlatformInterface> objA =
TemplateFactory<mviz::platform::PlatformInterface>::GetTypeInstance("j3");
std::unique_ptr<mviz::platform::PlatformInterface> objB =
TemplateFactory<mviz::platform::PlatformInterface>::GetTypeInstance("tda4"); void dynamicInitialization() {
  TemplateFactory<mviz::platform::PlatformInterface>::registerType("j3", []() { return std::make_unique<J3Paltform>();
}); TemplateFactory<mviz::platform::PlatformInterface>::registerType("tda4", []() { return
std::make_unique<Tda4VmPlatform>(); });
}

*/

}  // namespace mviz::utils

#endif