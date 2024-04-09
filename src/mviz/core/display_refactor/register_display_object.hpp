#ifndef REGISTER_DISPLAY_OBJECT_H_
#define REGISTER_DISPLAY_OBJECT_H_
#include <tuple>

#include "template_factory.hpp"

// todo::可变模板 + 函数元组模板递归(偏特化)，获取每个元组的类型
template <typename Tuple, size_t N>
struct RegisterObjects {
  static void Register() {
    RegisterObjects<Tuple, N - 1>::Register();
    using ElementType = std::tuple_element<N - 1, Tuple>::type;
    TemplateFactory<DispalyBase>::registerType("gridmap", []() { return std::make_unique<ElementType>(); });

    TemplateFactory<DispalyBase>::registerType("parkingspace",
                                               []() { return std::make_unique<ParkingspaceMakeker>(); });

  }
};
template <typename Tuple, 0>
struct RegisterObjects {
  static void Register() { std::cout << "register end" << std::endl; }
};

template <typename... DisplayTypes>
class RegisterDisplayObject {
 private:
  /* data */
 public:
  RegisterDisplayObject(/* args */) {
    // 获取每个元素的类型
    using DisplayType = std::tuple<DisplayTypes...>;
    RegisterObjects<DisplayType, sizeof...(DisplayTypes)>::Register();
  }
  ~RegisterDisplayObject() = default;
};

#endif