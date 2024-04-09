
#include <ros/ros.h>

#include <functional>
#include <iostream>
#include <map>
#include <memory>

#ifdef USED_CTRL_STOP
#include <csignal> h
#endif
#include "glog/logging.h"
#include "mviz_inf.h"
#include "version.h"
#ifdef USED_CTRL_STOP
static bool kStopMain = false;

void SingleHandler(int sig) {
  std::cout << ("\nleaving args_main\n") << std::endl;
  kStopMain = true;
}
#endif

std::map<std::string, ros::console::levels::Level> log_level_map{{"DEBUG", ros::console::levels::Debug},
                                                                 {"INFO", ros::console::levels::Info},
                                                                 {"WARN", ros::console::levels::Warn},
                                                                 {"ERROR", ros::console::levels::Error},
                                                                 {"FATAL", ros::console::levels::Fatal}};

int main(int argc, char *argv[]) {
  std::string platform = "havp_vm";

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 0;

  std::cout << "\033[32m===========================" << std::endl;
  std::cout << "mviz_" << platform << "_v" << _RELEASE_VERSION_ << "." << _MAJOR_VERSION_ << "." << _MINOR_VERSION_
            << "." << ((_MICRO_VERSION_ > 0) ? _MICRO_VERSION_ : "") << std::endl;
  std::cout << "===========================\033[0m" << std::endl;

  mviz_collect_init1();  //'0':hil ; '1':oncar
  mviz_collect_start();
  ros::spin();
  mviz_collect_stop();
  mviz_collect_release();
  ros::shutdown();

  return 0;
}
