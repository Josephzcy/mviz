#include "frames_pub_manager.h"

#include "display/pb_to_image.hpp"   
#include "display/pb_to_rosmsg.hpp"  

using namespace mviz1::data_conversion;
using namespace mviz1::data_visualization;
namespace mviz::replay {

struct PointCompare {
  bool operator()(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
    if (p1.y < p2.y)
      return true;
    else if (p1.y == p2.y && p1.x < p2.x)
      return true;
    else if (p1.y == p2.y && p1.x == p2.x && p1.z < p2.z)
      return true;
    return false;
  }
};

struct Point3DHash {
  std::size_t operator()(const geometry_msgs::Point& p) const {
    std::size_t h1 = std::hash<double>{}(p.x);
    std::size_t h2 = std::hash<double>{}(p.y);
    std::size_t h3 = std::hash<double>{}(p.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);  // Combine the hash values
  }
};

struct Point3DEqual {
  bool operator()(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
  }
};

// static std::set<geometry_msgs::Point, PointCompare> path_points;
// static std::set<geometry_msgs::Point, Point3DHash, Point3DEqual> path_points;

FramesPubManager::FramesPubManager(/* args */) : it_(nh_), private_nh("~") {
  pub_config_ = g_mrcfg->pub_manager_config;
  InitPublish();
  path.header.frame_id = "odome";
  path.header.stamp = ros::Time::now();

  path_.header.frame_id = "odome";
  path_.ns = "line_strip";
  path_.action = visualization_msgs::Marker::ADD;
  path_.type = visualization_msgs::Marker::POINTS;
  path_.scale.x = 0.1;
  path_.scale.y = 0.1;
  // path_.scale.z = 0.1;
  path_.color.r = 0.0;
  path_.color.g = 0.0;
  path_.color.b = 1.0;
  path_.color.a = 1.0;
  path_.pose.orientation.x = 0.0;
  path_.pose.orientation.y = 0.0;
  path_.pose.orientation.z = 0.0;
  path_.pose.orientation.w = 1.0;

  carModel =
      std::make_shared<GenerateCarModel>(pub_config_.car_body_dir, pub_config_.marker_radio, pub_config_.marker_scale);
  carModel->MakeCarModel(carMarker);
  managerMarker = std::make_shared<ManagerMarker>(pub_config_.marker_scale);
  image_front_position = g_mrcfg->camera_stitching_config.image_front_position;
  image_rear_position = g_mrcfg->camera_stitching_config.image_rear_position;
  image_left_position = g_mrcfg->camera_stitching_config.image_left_position;
  image_right_position = g_mrcfg->camera_stitching_config.image_right_position;

  std::string car_info_path = g_mrcfg->reader_manager_config.mviz_data_path + "/test_data/avm_calibs/bev_car_info.json";
  odom2bev_pix = std::make_shared<module::sensor_fusion::common::BevPix2Odom>(bev_width, bev_height, 6, car_info_path);
}

void FramesPubManager::InitPublish() {
  ROS_INFO_STREAM(__FUNCTION__ << ",start");
  // publish rosmsg for rviz visualization
  imagePubLeftUp_ = it_.advertise("fish_camera1", 10);
  imagePubLeftDown_ = it_.advertise("fish_camera2", 10);
  imagePubRightUp_ = it_.advertise("fish_camera3", 10);
  imagePubRightDown_ = it_.advertise("fish_camera4", 10);
  imageBevPub_ = it_.advertise("bird_view_img", 10);

  clearMakerPub = nh_.advertise<visualization_msgs::MarkerArray>("/viz/text", 10);
  carMarkerPub = nh_.advertise<visualization_msgs::MarkerArray>("/viz/car_model2D", 10);

  parkingOdometryVizPub = nh_.advertise<nav_msgs::Path>("/viz/odometry_3d", 10);
  parkingOdometryVizPub2 = nh_.advertise<visualization_msgs::Marker>("/viz/odometry_3d_2", 10);

  gridmapVizPub = nh_.advertise<visualization_msgs::Marker>("/viz/gridmap", 10);

  parkingspaceVizPub = nh_.advertise<visualization_msgs::MarkerArray>("/viz/parkingSpace", 10);
  uss_parkingspace_publish = nh_.advertise<visualization_msgs::MarkerArray>("/viz/uss_parkingspace", 10);

  planingVizPub = nh_.advertise<visualization_msgs::MarkerArray>("/viz/planing", 10);
  planningStateVizPub = nh_.advertise<visualization_msgs::Marker>("/viz/planningState", 10);
  vehicleControlVizPub = nh_.advertise<visualization_msgs::Marker>("/viz/vehicleControl", 10);

  // publish rosmsg for rviz pannel
  apaStateMsgPub = nh_.advertise<mviz_apa_show::ApaStateControl>("/msg/apa_state_control", 10);
  vehicleSignalMsgPub = nh_.advertise<mviz_apa_show::VehicleSignal>("/msg/vehicle_signal", 10);
  controlMsgPub = nh_.advertise<mviz_apa_show::VehicleControl>("/msg/vehicle_control", 10);
  uss_radar_publish_ = nh_.advertise<mviz_apa_show::UssRadar>("/qt_msg/uss_radar", 10);
  parkingTargetPub = nh_.advertise<visualization_msgs::MarkerArray>("/viz/parking_target", 10);
  ROS_INFO_STREAM(__FUNCTION__ << ",finish");
  return;
}

int FindTopicData(L2DataFramePtrList frames_ptr, std::string topic) {
  for (size_t i = 0; i < frames_ptr.size(); i++) {
    if (frames_ptr[i]->topic == topic) {
      return i;
    }
  }
  return -1;
}

void FramesPubManager::PublishFrame(L2DataFramePtrList frames_ptr, int seq) {
  if (frames_ptr.empty()) {
    return;
  }
  m_seq = seq;
  pub_timestamp_ = ros::Time::now();
  auto ret_index = -1;
  std::string topic;

  //【1】
  // "camera_stitching" + "freespace"
  topic = "camera_stitching";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    PubFishImgAndFreespace(frames_ptr, ret_index);
  }

  //【2】
  // "bev" + "raw_ins_ps" + "parkingspace_mviz"
  topic = "bev_libflow";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    PubBirdviewImg(frames_ptr, ret_index);
  }

  //【3】
  topic = "gridmap";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    perception::FreespaceObstacles pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    managerMarker->UpdateGridMapMarker(pb);
    gridmapVizPub.publish(managerMarker->grid_map_marker_);
  }

  //【4】
  topic = "parkingspace";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    perception::ParkingSpace pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    managerMarker->UpdateParkingSpaceMarker(pb);
    parkingspaceVizPub.publish(managerMarker->parkingspace_markers_);
  }

  //【5】
  topic = "planning_to_hmi";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::PlanningToHMI pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    managerMarker->UpdateSlotsRecommendAtt(pb);
    parkingspaceVizPub.publish(managerMarker->parkingspace_markers_);

    managerMarker->UpdatePlanningState(pb.remain_distance(), pb.gear());
    planningStateVizPub.publish(managerMarker->planning_state_marker_);
  }

  //【6】
  topic = "apa_state_control";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::APAStateControl pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    managerMarker->UpdateSlotsSelectedAtt(pb);
    parkingspaceVizPub.publish(managerMarker->parkingspace_markers_);
  }

  //【7】
  topic = "odometry_3d";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::Odometry3D pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    HandleOdometry3D(pb);
  }

  //【8】
  topic = "planning";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::Planning pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    managerMarker->UpdatePlanningMarker(pb, carModel->m_width);
    planingVizPub.publish(managerMarker->planning_markers_);
  }

  //【9】
  topic = "vehicle_control";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::VehicleControl pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    ControlData2RosMsg(pb, vehicleControl_);
    controlMsgPub.publish(vehicleControl_);
  }

  //【10】
  topic = "vehicle_signal";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::VehicleSignal pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    VehicleSignal2RosMsg(pb, vehicleSignal_);
    vehicleSignalMsgPub.publish(vehicleSignal_);
  }

  //【11】
  topic = "uss";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::UltrasonicRadar pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    UssRadar2RosMsg(pb, uss_radar_);
    uss_radar_publish_.publish(uss_radar_);
  }

  //【12】
  topic = "uss_parkingspace";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    perception::ParkingSpace pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    managerMarker->UpdateUssParkingspaceMarker(pb);
    uss_parkingspace_publish.publish(managerMarker->uss_parkingspace_markers_);
  }

  // 【13】
  topic = "parking_target";
  ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::parking::ObjectTrackList pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    managerMarker->UpdateParkingTargetMarker(pb);
    parkingTargetPub.publish(managerMarker->parking_target_markers_);
  }

  // test
  // minieye::parking::ObjectTrackList pb;
  // managerMarker->UpdateParkingTargetMarker(pb);
  // parkingTargetPub.publish(managerMarker->parking_target_markers_);
  // test end

  return;
}

// 输入四路鱼眼拼接，剪开，并将freespace(图像坐标系的)画上，发布图像
int FramesPubManager::PubFishImgAndFreespace(L2DataFramePtrList frames_ptr, int idx) {
  auto frame_ptr = frames_ptr[idx];
  int32_t imgWidth = frame_ptr->bgr.cols;
  int32_t imgHeight = frame_ptr->bgr.rows;
  if (frame_ptr->bgr.rows == 0 || frame_ptr->bgr.cols == 0) {
    return -1;
  }
  /*放到pub中去也可*/
  // 4分割
  cv::Rect segRoi[4]{cv::Rect(0, 0, int(imgWidth / 2), imgHeight / 2),
                     cv::Rect(0, imgHeight / 2, int(imgWidth / 2), imgHeight / 2),
                     cv::Rect(int(imgWidth / 2), 0, int(imgWidth / 2), imgHeight / 2),
                     cv::Rect(int(imgWidth / 2), imgHeight / 2, int(imgWidth / 2), imgHeight / 2)};
  //@Notice Mat名称不想改了，从上往下对应 前后左右。
  cv::Mat imgLeftUp = frame_ptr->bgr(segRoi[image_front_position - 1]);
  cv::Mat imgLeftDown = frame_ptr->bgr(segRoi[image_rear_position - 1]);
  cv::Mat imgRightUp = frame_ptr->bgr(segRoi[image_left_position - 1]);
  cv::Mat imgRightDown = frame_ptr->bgr(segRoi[image_right_position - 1]);

  // // 显示seq
  int imgSeq = frame_ptr->info.g_index;
  auto tick = frame_ptr->info.tick;  // us
  int fontFace = cv::FONT_HERSHEY_SIMPLEX || cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.5;
  cv::Scalar color = cv::Scalar(12, 12, 200);
  int thickness = 3;
  cv::putText(imgLeftUp, "frontCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgLeftDown, "rearCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgRightUp, "leftCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgRightDown, "rightCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgLeftDown, "Seq " + std::to_string(imgSeq) + " tick/us " + tick, cv::Point(40, 120), fontFace,
              fontScale, color, thickness);
  cv::putText(imgRightUp, "Seq " + std::to_string(imgSeq) + " tick/us " + tick, cv::Point(40, 120), fontFace, fontScale,
              color, thickness);
  cv::putText(imgLeftUp, "Seq " + std::to_string(imgSeq) + " tick/us " + tick, cv::Point(40, 120), fontFace, fontScale,
              color, thickness);
  cv::putText(imgRightDown, "Seq " + std::to_string(imgSeq) + " tick/us " + tick, cv::Point(40, 120), fontFace,
              fontScale, color, thickness);

  // 显示图像freespace
  // 点的vector：0 1 2 3 <--->前后左右
  auto ret = -1;
  ret = FindTopicData(frames_ptr, "freespace");
  if (ret != -1) {
    freespacepoints::FreespacePoints fspMsgCurr;
    fspMsgCurr.ParseFromArray(frames_ptr[ret]->data.data(), frames_ptr[ret]->data.size());
    if (fspMsgCurr.freespacepoints_size() >= 4) {
      DrawFreeSpace(imgLeftUp, fspMsgCurr.freespacepoints(0));
      DrawFreeSpace(imgLeftDown, fspMsgCurr.freespacepoints(1));
      DrawFreeSpace(imgRightUp, fspMsgCurr.freespacepoints(2));
      DrawFreeSpace(imgRightDown, fspMsgCurr.freespacepoints(3));
    }
  }

  // todo::publish todo FishEye image
  std_msgs::Header head;
  head.seq = imgSeq;
  head.stamp = pub_timestamp_;
  head.frame_id = "map";
  cv::Mat imgLeftUpResize;
  cv::Mat imgLeftDownResize;
  cv::Mat imgRightUpResize;
  cv::Mat imgRightDownResize;
  cv::resize(imgLeftUp, imgLeftUpResize, cv::Size(), 0.5, 0.5);
  cv::resize(imgLeftDown, imgLeftDownResize, cv::Size(), 0.5, 0.5);
  cv::resize(imgRightUp, imgRightUpResize, cv::Size(), 0.5, 0.5);
  cv::resize(imgRightDown, imgRightDownResize, cv::Size(), 0.5, 0.5);
  sensor_msgs::ImagePtr msgLU = cv_bridge::CvImage(head, "bgr8", imgLeftUpResize).toImageMsg();
  sensor_msgs::ImagePtr msgLD = cv_bridge::CvImage(head, "bgr8", imgLeftDownResize).toImageMsg();
  sensor_msgs::ImagePtr msgRU = cv_bridge::CvImage(head, "bgr8", imgRightUpResize).toImageMsg();
  sensor_msgs::ImagePtr msgRD = cv_bridge::CvImage(head, "bgr8", imgRightDownResize).toImageMsg();

  imagePubLeftUp_.publish(msgLU);
  imagePubLeftDown_.publish(msgLD);
  imagePubRightUp_.publish(msgRU);
  imagePubRightDown_.publish(msgRD);
  ROS_INFO_STREAM("publish fish image seq:" << imgSeq);
  return 0;
}

int FramesPubManager::PubBirdviewImg(L2DataFramePtrList frames_ptr, int idx) {
  auto bev_frame = frames_ptr[idx];

  auto ret = -1;

  if (bev_frame->bgr.rows == 0 || bev_frame->bgr.cols == 0) {
    return -1;
  }

  ret = FindTopicData(frames_ptr, "raw_ins_ps");
  if (ret != -1) {
    perception::RawInsParkingSpace raw_ins_pts;
    raw_ins_pts.ParseFromArray(frames_ptr[ret]->data.data(), frames_ptr[ret]->data.size());
    DrawRawInsParkingSpace(bev_frame->bgr, raw_ins_pts);
  }

  ret = FindTopicData(frames_ptr, "parkingspace_mviz");
  if (ret != -1) {
    perception::ParkingSpace park_sp;
    park_sp.ParseFromArray(frames_ptr[ret]->data.data(), frames_ptr[ret]->data.size());
    DrawParkingSpace(bev_frame->bgr, park_sp);
  }

  ret = FindTopicData(frames_ptr, "parkingspace");
  if (ret != -1) {
    perception::ParkingSpace park_sp;
    park_sp.ParseFromArray(frames_ptr[ret]->data.data(), frames_ptr[ret]->data.size());
    UpdateParkSpaceSet(park_sp);
  }

  auto topic = "odometry_3d";
  auto ret_index = FindTopicData(frames_ptr, topic);
  if (ret_index != -1) {
    minieye::Odometry3D pb;
    pb.ParseFromArray(frames_ptr[ret_index]->data.data(), frames_ptr[ret_index]->data.size());
    DrawOdoParkingSpaceOnBev(pb, bev_frame->bgr);
  }

  int fontFace = cv::FONT_HERSHEY_SIMPLEX || cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.8;
  cv::Scalar color = cv::Scalar(12, 12, 200);
  int thickness = 2;
  cv::putText(bev_frame->bgr, "Bev Seq " + std::to_string(bev_frame->info.g_index), cv::Point(10, 40), fontFace,
              fontScale, color, thickness);

  cv::putText(bev_frame->bgr, "tick/us " + bev_frame->info.tick, cv::Point(10, 80), fontFace, fontScale, color,
              thickness);

  std_msgs::Header head;
  head.seq = bev_frame->info.g_index;
  head.stamp = pub_timestamp_;
  head.frame_id = "map";

  cv::Rect roi(0, 0, 352, 480);
  // qDebug() << "bev id:" << head.seq << ",h*w:" <<bev_frame->bgr.rows << "," << bev_frame->bgr.cols;

  sensor_msgs::ImagePtr msgBv = cv_bridge::CvImage(head, "bgr8", bev_frame->bgr(roi)).toImageMsg();
  imageBevPub_.publish(msgBv);
  return 0;
}

void FramesPubManager::HandleOdometry3D(minieye::Odometry3D& parkingOdometryMsg) {
  if (parkingOdometryMsg.distance() < 0.5) {
    this->clearMarkerFlag = true;
  } else {
    this->clearMarkerFlag = false;
  }
  tf::Transform transform;
  double x = parkingOdometryMsg.pose().x();
  double y = parkingOdometryMsg.pose().y();
  double z = parkingOdometryMsg.pose().z();
  double roll = parkingOdometryMsg.pose().roll();
  double pitch = parkingOdometryMsg.pose().pitch();
  double yaw = parkingOdometryMsg.pose().yaw();

  transform.setOrigin(tf::Vector3(x, y, z));
  tf::Quaternion q;

  q.setRPY(roll, pitch, yaw);

  q.normalize();
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odome", "base_link"));
  // todo:odome 画车位置
  geometry_msgs::PoseStamped currentPose;
  currentPose.pose.position.x = x;
  currentPose.pose.position.y = y;
  currentPose.pose.position.z = z;
  currentPose.pose.orientation.x = q.getX();
  currentPose.pose.orientation.y = q.getY();
  currentPose.pose.orientation.z = q.getZ();
  currentPose.pose.orientation.w = q.getW();
  currentPose.header.frame_id = "odome";
  currentPose.header.stamp = ros::Time::now();
  path.poses.push_back(currentPose);
  parkingOdometryVizPub.publish(path);
  carMarkerPub.publish(carMarker);

  geometry_msgs::Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  path_points[m_seq] = pt;
  path_.points.clear();
  for (const auto& pt : path_points) {
    path_.points.push_back(pt.second);
  }
  path_.header.stamp = ros::Time::now();
  parkingOdometryVizPub2.publish(path_);
}

void FramesPubManager::UpdateParkSpaceSet(perception::ParkingSpace& pb) {
  for (const auto& parkslots : pb.parkslots()) {
    auto size = parkslots.corner_pts_size();
    if (size != 4) {
      continue;
    }
    odo2pix_park_spaces_map[parkslots.id()] = parkslots;
  }
}

void FramesPubManager::DrawOdoParkingSpaceOnBev(const minieye::Odometry3D& pb, cv::Mat& img) {
  if (odo2pix_park_spaces_map.empty()) {
    return;
  }

  for (const auto& parkslot : odo2pix_park_spaces_map) {
    std::vector<cv::Point> rect;
    // std::cout << "id: " << parkslot.first << std::endl;
    for (int i = 0; i < 4; ++i) {
      Eigen::Matrix<double, 3, 1> arr;
      arr << parkslot.second.corner_pts(i).odom_pt().x(), parkslot.second.corner_pts(i).odom_pt().y(), 0;
      module::sensor_fusion::common::Point3D odom_pt = arr;
      module::sensor_fusion::common::Pose3D pose;
      module::sensor_fusion::common::Odom3DToPose3D(pb, &pose);
      cv::Point pt_pix = odom2bev_pix->Odom2Pix(odom_pt, pose);
      rect.push_back(pt_pix);
      // std::cout << "[" << i << "]odom:" << odom_pt.transpose() << ",pix: " << pt_pix << std::endl;
    }
    if (!rect.empty()) {
      if (parkslot.second.occupied()) {
        cv::polylines(img, rect, true, cv::Scalar(0.56 * 255, 0 * 255, 1.0 * 255), 2);

      } else {
        cv::polylines(img, rect, true, cv::Scalar(1 * 255, 1 * 255, 0 * 255), 2);
      }
    }
  }
}
FramesPubManager::~FramesPubManager() {}

}  // namespace mviz::replay
