#ifndef L2_FRAMES2ROSMAG_PUB_H
#define L2_FRAMES2ROSMAG_PUB_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <array>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <unordered_set>
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

// rosmsg
#include "bev_pix2odom.h"
#include "display/generate_car_model.h"
#include "global_value.h"
#include "l2_types.h"
#include "display/manager_marker.h"
#include "mviz_apa_show/ApaStateControl.h"
#include "mviz_apa_show/PlanningToHMI.h"
#include "mviz_apa_show/UssRadar.h"
#include "mviz_apa_show/VehicleControl.h"
#include "mviz_apa_show/VehicleSignal.h"
#include "display/viz_struct.hpp"

namespace mviz::replay {
using MatrixArray = std::array<Eigen::Matrix<double, 3, 1>, 4>;  // 用来存放一个车位的四个点

struct MatrixArrayComparator {
  bool operator()(const MatrixArray &arr1, const MatrixArray &arr2) const {
    for (int i = 0; i < 4; ++i) {
      auto x1 = arr1[i](0, 0);
      auto y1 = arr1[i](0, 1);
      auto z1 = arr1[i](0, 2);

      auto x2 = arr2[i](0, 0);
      auto y2 = arr2[i](0, 1);
      auto z2 = arr2[i](0, 2);

      if (x1 < x2 || (x1 == x2 && y1 < y2) || (x1 == x2 && y1 == y2 && z1 < z2)) {
        return true;
      }
    }
    return false;
  }
};

using ParkSpaceSet = std::set<MatrixArray, MatrixArrayComparator>;
class FramesPubManager {
 private:
  int m_seq{0};
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
  ros::Publisher parkingOdometryVizPub2;
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
  visualization_msgs::Marker path_;
  std::map<int, geometry_msgs::Point> path_points;  // <seq, point>
  std::shared_ptr<GenerateCarModel> carModel;
  visualization_msgs::MarkerArray carMarker;
  std::shared_ptr<ManagerMarker> managerMarker;

  mviz_apa_show::VehicleSignal vehicleSignal_;
  mviz_apa_show::VehicleControl vehicleControl_;
  mviz_apa_show::ApaStateControl apaStateControl_;
  mviz_apa_show::UssRadar uss_radar_;

  PubManagerConfig pub_config_;
  /* 图像拼接位置ID
   1 | 3
  -------
   2 | 4
  */
  int image_front_position;
  int image_rear_position;
  int image_left_position;
  int image_right_position;
  bool clearMarkerFlag{false};

 public:
  FramesPubManager();
  ~FramesPubManager();
  void InitPublish();
  void PublishFrame(L2DataFramePtrList, int);
  int PubFishImgAndFreespace(L2DataFramePtrList, int);
  int PubBirdviewImg(L2DataFramePtrList, int);
  void HandleOdometry3D(minieye::Odometry3D &);

  ros::Time pub_timestamp_;

 private:
  int bev_width = 352;
  int bev_height = 480;
  std::shared_ptr<module::sensor_fusion::common::BevPix2Odom> odom2bev_pix;
  // ParkSpaceSet odo2pix_park_spaces_set;
  std::map<int, perception::ParkingSlot> odo2pix_park_spaces_map;
  void UpdateParkSpaceSet(perception::ParkingSpace &park_sp);
  void DrawOdoParkingSpaceOnBev(const minieye::Odometry3D &pb, cv::Mat &);
};

}  // namespace mviz::replay

#endif

/*
1、暂停时，3D坐标的元素不能一直停留
2、后退时，发布的时间戳是递增的，rviz显示可能会有问题
*/
