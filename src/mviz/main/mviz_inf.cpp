
// Created by nick on 23-2-13.
//

#include "mviz_inf.h"

#include "collect/topic_global_object_register.hpp"
#include "display/display_srv.hpp"  //
using namespace minieye::mviz::config;
using namespace minieye::mviz::collect;

using minieye::mviz::collect::CDataCollectDBMan;
using minieye::mviz::collect::CDataCollectSrv;
using namespace minieye::mviz::display;

CDataCollectSrv *CDataCollectSrv::pSingletonCollectSrv = new CDataCollectSrv;
CDataCollectDBMan *CDataCollectDBMan::pSingletonCollectDBMan = new CDataCollectDBMan;

static std::shared_ptr<DisplaySrv> pMvizDisplaySrv = std::make_shared<DisplaySrv>();

// init the mviz collect
MVIZ_COLLECT_API int mviz_collect_init(int mode) {
  if (CDataCollectSrv::get_instance() == nullptr || CDataCollectDBMan::get_instance() == nullptr) {
    DLOG(ERROR) << "mviz collect init error and point is nullptr";
    return -1;
  }
  std::vector<std::string> topicVec;
  CDataCollectSrv::get_instance()->get_collect_config_man().init(mode);
  if (CDataCollectSrv::get_instance()->load_params(topicVec) != 0)
    LOG(ERROR) << "mviz collect init error and was inited";

  if (!CDataCollectDBMan::get_instance()->init_to_dbm(topicVec))
    LOG(ERROR) << "mviz collect-dbm init error and was inited";

  LOG(INFO) << "mviz collect [" << CDataCollectSrv::get_instance() << "] and dbm [" << CDataCollectDBMan::get_instance()
            << "] --init ok";
  return 0;
}

MVIZ_COLLECT_API int mviz_collect_init1() {
  // Initialize ROS system
  ros::init(ros::M_string(), "display_node");
  ros::NodeHandle nh;
  std::string config_path,mviz_mode;
  nh.param<std::string>("/config_dir", config_path, "src/mviz/config/mviz_config.json");
  nh.param<std::string>("/mviz_mode", mviz_mode, "oncar");


  std::vector<std::string> topicVec;
  CDataCollectSrv::get_instance()->get_collect_config_man().init(config_path,mviz_mode);
  if (CDataCollectSrv::get_instance()->load_params(topicVec) != 0)
    LOG(ERROR) << "mviz collect init error and was inited";

  if (!CDataCollectDBMan::get_instance()->init_to_dbm(topicVec))
    LOG(ERROR) << "mviz collect-dbm init error and was inited";

  LOG(INFO) << "mviz collect [" << CDataCollectSrv::get_instance() << "] and dbm [" << CDataCollectDBMan::get_instance()
            << "] --init ok";

  if (pMvizDisplaySrv == nullptr) {
    LOG(ERROR) << "mviz display sever init error and pointer is nullptr";
    return -1;
  }
  pMvizDisplaySrv->init();
  if (CDataCollectSrv::get_instance() == nullptr || CDataCollectDBMan::get_instance() == nullptr) {
    LOG(ERROR) << "mviz collect init error and point is nullptr";
    return -1;
  }
  return 0;
}

MVIZ_COLLECT_API int mviz_collect_start() {
  auto ret = -1;
  auto ptr = CDataCollectSrv::get_instance();
  LOG(INFO) << "mviz collect starting...... ";

  if (ptr == nullptr) {
    LOG(WARNING) << "mviz collect not init or EMPTY ";
    return -1;
  }
  ret = CDataCollectSrv::get_instance()->start();

  ret == 0 ? LOG(INFO) << "mviz collect srv start is succeed " : LOG(ERROR) << "mviz collect srv start error ";

  // 启动显示相关的线程
  pMvizDisplaySrv->start();
  return ret;
}

MVIZ_COLLECT_API int mviz_collect_stop() {
  LOG(INFO) << "mviz collect will stopping...... ";
  if (CDataCollectSrv::get_instance() == nullptr) {
    LOG(WARNING) << "mviz collect not init or EMPTY ";
    return -1;
  }
  auto ret = CDataCollectSrv::get_instance()->stop();  // 不再保存订阅的数据
  CDataCollectDBMan::get_instance()->drop_data();      // 清除缓存

  ret == 0 ? LOG(INFO) << "mviz collect srv stop is succeed " : LOG(ERROR) << "mviz collect srv stop error ";

  pMvizDisplaySrv->release();
  return ret;
}

MVIZ_COLLECT_API int mviz_collect_release() {
  LOG(INFO) << "release mviz collect module is starting...";
  if (CDataCollectDBMan::get_instance() != nullptr) {
    CDataCollectDBMan::delete_instance();  // add the stop
    LOG(INFO) << "release mviz collect-db point is " << CDataCollectDBMan::get_instance();
  }

  if (CDataCollectSrv::get_instance() != nullptr) {
    CDataCollectSrv::delete_instance();
    LOG(INFO) << "release mviz collect module point is " << CDataCollectSrv::get_instance();
  }

  LOG(INFO) << "release mviz collect and dbm srv was released...";

  if (pMvizDisplaySrv == nullptr) {
    LOG(ERROR) << "mviz display sever release error ";
    return -1;
  }
  pMvizDisplaySrv->release();

  return 0;
}