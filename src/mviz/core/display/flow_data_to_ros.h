// Copyright 2023 MINIEYE
#ifndef FLOW_DATA_TO_ROS_H_
#define FLOW_DATA_TO_ROS_H_
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

// aiplorer
#include "msgpack.hpp"
// pb
#include "RawInsParkingSpace.pb.h"
#include "apa_state.pb.h"
#include "freespace_obstacles.pb.h"
#include "freespacepoints.pb.h"
#include "odometry_3d.pb.h"
#include "parking_object.pb.h"
#include "parkingspace.pb.h"
#include "planning.pb.h"
#include "planningtohmi.pb.h"
#include "ultra_radar.pb.h"
#include "vehicle_control.pb.h"
#include "vehicle_signal.pb.h"

#include "ap_map_response.pb.h"
#include "planning.pb.h"
#include "collect/collect_dbm.hpp"
#include "decode_video.h"
#include "generate_car_model.h"
#include "manager_marker.h"
#include "viz_struct.hpp"


// rosmsg
#include "mviz_apa_show/ApaStateControl.h"
#include "mviz_apa_show/PlanningToHMI.h"
#include "mviz_apa_show/UssRadar.h"
#include "mviz_apa_show/VehicleControl.h"
#include "mviz_apa_show/VehicleSignal.h"

#include "mviz_apa_show/Planning.h"
#include "mviz_apa_show/ApMapResponse.h"

#include "ap_map_response.pb.h"
#include "map_engine_response.pb.h"

#include "map_engine_marker.h"
using namespace havp::ap_slam;
namespace minieye::mviz::display {
// libflow接收的消息转换成ros消息发布，在rviz中订阅显示
// ImgBirdview：融合鸟瞰图，显示车位
// ImgFish（4拼：左上 左下 右上 右下）：环视鱼眼图，显示感知结果
// 3D坐标：车身轨迹，规划路线，车位
class ApaData2Ros {
 public:
  explicit ApaData2Ros(int);
  explicit ApaData2Ros();
  ApaData2Ros(const ApaData2Ros &) = delete;
  ApaData2Ros &operator=(const ApaData2Ros &) = delete;
  ApaData2Ros(ApaData2Ros &&) = delete;
  ApaData2Ros &operator=(ApaData2Ros &&) = delete;
  ~ApaData2Ros();
  void start();
  void stop();
  int init();

 private:
  void LoadLaunchPara();
  void InitPublish();
  void InitMarker();

 private:
  void PublishFlowData();
  int PubFishImgAndFreespace();
  int PubBirdviewImg(uint32_t seq, uint64_t fishImageTick);
  bool PublishUssParkingSpace(std::uint64_t &fish_image_tick);
  int Pub3DCoordinate(std::uint64_t &fish_image_tick);
  void clearMarker(std::string frameid = "map");
  bool PubApmapResponseStatus(std::uint64_t &fish_image_tick);
  bool PubMapEngineResponse(std::uint64_t &fish_image_tick);


  void CacheFlowPbData();
  bool CacheImgData();
  bool CacheImgData(std::string topic, std::deque<ImageFlowHeaderStruct> &imgHeader,
                    std::deque<ImageFlowStruct> &imgFrames, decode_video_sw *imgDecoder);
  bool CacheImgData(std::string topic, std::deque<ImageFlowHeaderStruct> &imgHeader,
                    std::deque<ImageFlowStruct> &imgFrames, decode_video_hw *imgDecoder);
  void ClearCacheFlowPbData();

  std::string mviz_config_dir_;
  std::string mviz_replay_mode_;  // oncar / hil
  std::string folder_name_;

  bool bShowUssParkingspace_;
  bool _bWantStop{false};
  nlohmann::json hmiTime_json;
  std::map<std::string, FlowIPconfig> libflow_configs_map_;
  std::thread *pub_thread{nullptr};

  int show_wait_time_{0};
  // 微秒
  size_t cacheImageNumber;  //=1, 收到即显示；>1,缓存n帧之后再可视化。
  std::string m_odometry_type;
  /* 图像拼接位置ID
   1 | 3
  -------
   2 | 4
  */
  int image_front_position;
  int image_rear_position;
  int image_left_position;
  int image_right_position;
  // ros
  double pubRate;
  bool clearMarkerFlag{false};
  int testCount{0};
  bool firstLocationFlag{true};
  double carModelSize;

  int cache_tick_dt_ms_;
  int sync_tick_dt_ms_;

  // video decode
  bool useHwdecode{true};
  int codeType{AV_CODEC_ID_H264};  //  /*AV_CODEC_ID_H265 // AV_CODEC_ID_H264*/
  decode_video_sw *imgDecoderFishSW{nullptr};
  decode_video_hw *imgDecoderFishHW{nullptr};
  decode_video_sw *imgDecoderBevSW{nullptr};
  decode_video_hw *imgDecoderBevHW{nullptr};
  std::deque<ImageFlowHeaderStruct> imgBirdviewHeader;
  std::deque<ImageFlowHeaderStruct> imgFishHeader;

  // todo: show cache
  NewHilMsg srcMsg_;
  std::deque<ImageFlowStruct> imgFramesFish;
  std::deque<ImageFlowStruct> imgFramesBev;

  std::map<uint64_t, perception::ParkingSpace> parkingspaceDatasMap;
  std::map<uint64_t, freespacepoints::FreespacePoints> freespaceDatasMap;
  std::map<uint64_t, perception::RawInsParkingSpace> rawInsPsDatasMap;
  std::map<uint64_t, perception::FreespaceObstacles> gridmapDatasMap;
  std::map<uint64_t, perception::ParkingSpace> parkingSpaceMvizBuf;

  std::deque<minieye::APAStateControl> APAStateControlBuf;

  std::deque<minieye::Odometry3D> parkingOdometryMsgMap;
  std::deque<minieye::Planning> planningDatasMap;
  std::deque<minieye::VehicleControl> controlDatasMap;
  std::deque<minieye::PlanningToHMI> planningToHmiBuf;
  std::deque<minieye::PlanningToHMI> planningToHmiRvizBuf;
  std::deque<perception::ParkingSpace> uss_parkingspace_mviz_buffer_;
  std::deque<minieye::parking::ObjectTrackList> parkingTargetDatasQue;

  std::deque<minieye::parking::ApMapResponse> apmap_response_datas_quene_;
  std::deque<minieye::parking::MapEngineResponse> map_engine_response_datas_quene_;



  // todo:viz marker publish
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::NodeHandle private_nh;

  image_transport::Publisher imagePubLeftUp_;
  image_transport::Publisher imagePubLeftDown_;
  image_transport::Publisher imagePubRightUp_;
  image_transport::Publisher imagePubRightDown_;
  image_transport::Publisher imageBevPub_;

  ros::Publisher clearMakerPub;
  ros::Publisher carMarkerPub;
  ros::Publisher parkingOdometryVizPub;
  ros::Publisher gridmapVizPub;
  ros::Publisher parkingspaceVizPub;
  ros::Publisher uss_parkingspace_publish;
  ros::Publisher planingVizPub;
  ros::Publisher planningStateVizPub;
  ros::Publisher vehicleControlVizPub;
  ros::Publisher controlMsgPub;
  ros::Publisher apaStateMsgPub;
  ros::Publisher vehicleSignalMsgPub;
  ros::Publisher uss_radar_publish_;
  ros::Publisher parkingTargetPub;
  tf::TransformBroadcaster br;

  visualization_msgs::MarkerArray clearMarkers;
  nav_msgs::Path path;
  visualization_msgs::MarkerArray carMarker;


  // 类中指针变量直接初始化
  std::shared_ptr<GenerateCarModel> carModel  ;
  std::shared_ptr<ManagerMarker> managerMarker;
  std::shared_ptr<MapEngineMarker> map_engine_marker_;


  std::string carBodyDir;
  float markerScale;
  float markerRadio;

  float maker_lifetime_;

  // todo::rosmsg_to_gui
  mviz_apa_show::VehicleSignal vehicleSignal_;
  mviz_apa_show::VehicleControl vehicleControl_;
  mviz_apa_show::ApaStateControl apaStateControl_;
  mviz_apa_show::UssRadar uss_radar_;

  mviz_apa_show::Planning planning_;
  mviz_apa_show::ApMapResponse apmap_response_;

  ros::Publisher planning_to_qt_pub_;
  ros::Publisher apmap_response_to_qt_pub_;
  ros::Publisher map_engine_response_pub_;


};

}  // namespace minieye::mviz::display
#endif
