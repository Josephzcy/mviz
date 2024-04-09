#include <ros/ros.h>

#include <iostream>

#include "collect_dbm.hpp"
#include "flow_data_to_ros.h"

using minieye::mviz::collect::CDataCollectDBMan;
using std::cout;
using std::endl;
CDataCollectDBMan *CDataCollectDBMan::pSingletonCollectDBMan = new CDataCollectDBMan;

minieye::mviz::display::ApaData2Ros *node_name;

// SIGINT信号处理函数
// void sigintHandler(int sig)
// {
//     // 执行自定义的退出操作
//     ROS_INFO("Received SIGINT signal. Shutting down...");
//     node_name->stop();
//     ros::shutdown();
// }

int main(int argc, char *argv[]) {
  // 设置SIGINT信号处理函数
  // signal(SIGINT, sigintHandler);
  if (CDataCollectDBMan::get_instance() == nullptr) {
    ROS_ERROR_STREAM("mviz collect init error and point is nullptr");
    return -1;
  }
  ros::init(argc, argv, "display_test");
  std::vector<std::string> topicVec{"camera30", "bev_libflow", "vehicle_signal"};
  if (!CDataCollectDBMan::get_instance()->init_to_dbm(topicVec))
    ROS_ERROR_STREAM("mviz collect-dbm init error and was inited");
  node_name = new minieye::mviz::display::ApaData2Ros();
  node_name->start();

  ros::spin();
  ROS_INFO_STREAM("after spin");
  node_name->stop();
  CDataCollectDBMan::get_instance()->drop_data();  // 清除缓存
  if (CDataCollectDBMan::get_instance() != nullptr) {
    CDataCollectDBMan::delete_instance();  // add the stop
    ROS_ERROR_STREAM("release mviz collect-db point is " << CDataCollectDBMan::get_instance());
  }
  delete node_name;
  ros::shutdown();
  ROS_INFO_STREAM("display_test node finished");
  return 0;
}
