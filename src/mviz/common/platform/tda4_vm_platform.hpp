#ifndef TDA4_VM_PLATFORM_H_
#define TDA4_VM_PLATFORM_H_
#include <iostream>

#include "platform_interface.hpp"

namespace mviz::platform {
class Tda4VmPlatform : public PlatformInterface {
 private:
  /* data */
 public:
  Tda4VmPlatform(/* args */) = default;
  ~Tda4VmPlatform() = default;

 public:
  void virtual PerPbhandle() override { std::cout << "Tda4VmPlatform" << std::endl; }
};
}  // namespace mviz::platform
#endif