#ifndef J3_PLATFORM_H_
#define J3_PLATFORM_H_
#include "platform_interface.hpp"
namespace mviz::platform {
class J3Platform : public PlatformInterface {
 private:
  /* data */
 public:
  J3Platform(/* args */) = default;
  ~J3Platform() = default;

 public:
  void virtual PerPbhandle() override { std::cout << "J3Platform" << std::endl; }
};
}  // namespace mviz::platform
#endif