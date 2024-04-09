
#include <ros/ros.h>

#include <functional>
#include <iostream>
#include <map>
#include <memory>

#ifdef USED_CTRL_STOP
#include <csignal> h
#endif
#include "glog/logging.h"
// #include "mviz_inf.h"
// #include "performance_analyzer/performance_analyzer.h"
#include "bind_thread_to_cpu/bind_thread_to_cpu.h"
// #include "template_factory.hpp"
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
  // std::string log_level;
  // nh.getParam("log_level", log_level);
  // ROS_INFO_STREAM("log_level : " << log_level);
  // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, log_level_map[log_level]);

  // int ret = 0;
  // google::InitGoogleLogging(argv[0]);
  // google::InstallFailureSignalHandler();
  // FLAGS_logtostderr = true;
  // FLAGS_colorlogtostderr = true;
  // FLAGS_stderrthreshold = 0;

  // auto cpu_profile = utils::profile::PerformanceAnalyzer::CPUProfileAnalyzer("/root/mviz2_ros/mviz.prof");
  // auto time_profile = utils::profile::PerformanceAnalyzer::TimeProfileAnalysis("/root/mviz2_ros/time.txt");

  // mviz_collect_init1();  //'0':hil ; '1':oncar
  // mviz_collect_start();
  // ros::spin();
  // mviz_collect_stop();
  // mviz_collect_release();
  // ros::shutdown();
  // std::cout << "ros has been shutted down..." << std::endl;

  std::shared_ptr<utils::BindThreadToCpu>  ptrThreadCpu= std::make_shared<utils::BindThreadToCpu>();
  ptrThreadCpu->start();

  return 0;
}
