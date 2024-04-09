#ifndef DISPLAY_SRV_H_
#define DISPLAY_SRV_H_
#include <ros/ros.h>

#include "display/flow_data_to_ros.h"
namespace minieye::mviz::display {
class DisplaySrv {
 private:
  /* data */
  ApaData2Ros* pMvizDisplay{nullptr};

 public:
  DisplaySrv(/* args */) {}
  ~DisplaySrv() {}

 public:
  int init() {
    if (pMvizDisplay == nullptr) {
      pMvizDisplay = new ApaData2Ros();
    }
    pMvizDisplay->init();
    return 1;
  }
  int start() {
    if (pMvizDisplay != nullptr) {
      pMvizDisplay->start();
      return 0;
    }
    return 1;
  }
  int stop() {
    if (pMvizDisplay != nullptr) {
      pMvizDisplay->stop();
      return 0;
    }
    return 1;
  }

  int release() {
    if (pMvizDisplay != nullptr) {
      delete pMvizDisplay;
      pMvizDisplay = nullptr;
      return 0;
    }
    return 1;
  }
};

}  // namespace minieye::mviz::display
#endif