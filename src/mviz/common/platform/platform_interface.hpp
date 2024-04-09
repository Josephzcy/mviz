#ifndef PLATFORM_INTERFACE_H_
#define PLATFORM_INTERFACE_H_

namespace mviz::platform {

class PlatformInterface {
 private:
  /* data */
 public:
  PlatformInterface(/* args */) = default;
  ~PlatformInterface() = default;

 public:
  virtual void PerPbhandle() = 0;
};
}  // namespace mviz::platform

#endif