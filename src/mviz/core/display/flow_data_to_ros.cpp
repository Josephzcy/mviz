#include "display/flow_data_to_ros.h"

#include "pb_to_image.hpp"   // 需放在放在cpp中，不然会报错：重定义，原因不明
#include "pb_to_rosmsg.hpp"  // 需放在放在cpp中，不然会报错：重定义，原因不明
static std::string bev_topic = "bev_libflow";
using namespace minieye::mviz::display;

void ApaData2Ros::clearMarker(std::string frameid) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameid;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.frame_locked = false;
  clearMarkers.markers.push_back(marker);
  gridmapVizPub.publish(marker);
  path.poses.clear();
  clearMakerPub.publish(clearMarkers);
  return;
}

// todo::mviz_show
int ApaData2Ros::PubFishImgAndFreespace() {
  ImageFlowStruct &imgFrameCurr = imgFramesFish.front();
  int32_t imgWidth = imgFrameCurr.Head.Width;
  int32_t imgHeight = imgFrameCurr.Head.Height;
  /*放到pub中去也可*/
  // 4分割
  cv::Rect segRoi[4]{cv::Rect(0, 0, int(imgWidth / 2), imgHeight / 2),
                     cv::Rect(0, imgHeight / 2, int(imgWidth / 2), imgHeight / 2),
                     cv::Rect(int(imgWidth / 2), 0, int(imgWidth / 2), imgHeight / 2),
                     cv::Rect(int(imgWidth / 2), imgHeight / 2, int(imgWidth / 2), imgHeight / 2)};
  //@Notice Mat名称不想改了，从上往下对应 前后左右。
  cv::Mat imgLeftUp = imgFrameCurr.data(segRoi[image_front_position - 1]);
  cv::Mat imgLeftDown = imgFrameCurr.data(segRoi[image_rear_position - 1]);
  cv::Mat imgRightUp = imgFrameCurr.data(segRoi[image_left_position - 1]);
  cv::Mat imgRightDown = imgFrameCurr.data(segRoi[image_right_position - 1]);
  // std::vector<cv::Mat> imgSplited;
  // // 显示seq
  int imgSeq = imgFrameCurr.Head.Seq;
  int64_t tick = imgFrameCurr.Head.Sec * 1000000 + imgFrameCurr.Head.Nsec / 1000;  // us
  int fontFace = cv::FONT_HERSHEY_SIMPLEX || cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.5;
  cv::Scalar color = cv::Scalar(12, 12, 200);
  int thickness = 3;
  cv::putText(imgLeftUp, "frontCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgLeftDown, "rearCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgRightUp, "leftCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgRightDown, "rightCam", cv::Point(40, 80), fontFace, fontScale, color, thickness);
  cv::putText(imgLeftDown, "Seq " + std::to_string(imgSeq) + " tick/us " + std::to_string(tick), cv::Point(40, 120),
              fontFace, fontScale, color, thickness);
  cv::putText(imgRightUp, "Seq " + std::to_string(imgSeq) + " tick/us " + std::to_string(tick), cv::Point(40, 120),
              fontFace, fontScale, color, thickness);
  cv::putText(imgLeftUp, "Seq " + std::to_string(imgSeq) + " tick/us " + std::to_string(tick), cv::Point(40, 120),
              fontFace, fontScale, color, thickness);
  cv::putText(imgRightDown, "Seq " + std::to_string(imgSeq) + " tick/us " + std::to_string(tick), cv::Point(40, 120),
              fontFace, fontScale, color, thickness);
  // 显示图像freespace
  // 点的vector：0 1 2 3 <--->前后左右
  ROS_WARN_STREAM("freespaceDatasMap size:" << freespaceDatasMap.size());
  auto iterFsp = freespaceDatasMap.find(imgSeq);
  if (iterFsp != freespaceDatasMap.end()) {
    freespacepoints::FreespacePoints fspMsgCurr = iterFsp->second;

    if (fspMsgCurr.freespacepoints_size() >= 4) {
      mviz1::data_visualization::DrawFreeSpace(imgLeftUp, fspMsgCurr.freespacepoints(0));
      mviz1::data_visualization::DrawFreeSpace(imgLeftDown, fspMsgCurr.freespacepoints(1));
      mviz1::data_visualization::DrawFreeSpace(imgRightUp, fspMsgCurr.freespacepoints(2));
      mviz1::data_visualization::DrawFreeSpace(imgRightDown, fspMsgCurr.freespacepoints(3));
      std::uint64_t inferenceTick = iterFsp->second.tick();
      ROS_DEBUG_STREAM("freespace tick :" << inferenceTick);

      auto gridmapIter = gridmapDatasMap.find(inferenceTick);
      if (gridmapIter != gridmapDatasMap.end()) {
        ROS_DEBUG_STREAM("gridmap correspond to inference tick :" << gridmapIter->first);
        perception::FreespaceObstacles &gridmap = gridmapIter->second;
        if (managerMarker->UpdateGridMapMarker(gridmap)) {
          // todo::gridmap in odome corodinate in condition that gridmap find
          // tick of inference
          gridmapVizPub.publish(managerMarker->grid_map_marker_);
          gridmapDatasMap.erase(gridmapIter);
        }
      }
    }
    freespaceDatasMap.erase(iterFsp);
  }

  // todo::publish todo FishEye image
  std_msgs::Header head;
  head.seq = imgFrameCurr.Head.Seq;
  head.stamp = imgFrameCurr.timestamp;
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
  ROS_INFO_STREAM("publish fish image seq:" << imgFrameCurr.Head.Seq);
  imgFramesFish.pop_front();
  return 0;
}

int ApaData2Ros::PubBirdviewImg(uint32_t imgSeq, uint64_t imgTickms) {
  // imgFramesBev find imgSeq; erase
  auto bev_image_iter = imgFramesBev.begin();
  for (; bev_image_iter != imgFramesBev.end();) {
    if (bev_image_iter->Head.Seq < imgSeq) {
      bev_image_iter = imgFramesBev.erase(bev_image_iter);
    } else if (bev_image_iter->Head.Seq == imgSeq) {
      break;
    } else {
      bev_image_iter = imgFramesBev.end();
      break;
    }
  }

  if (bev_image_iter != imgFramesBev.end()) {
    std::uint64_t bev_image_tick = bev_image_iter->Head.Sec * 1e3 + bev_image_iter->Head.Nsec * 1e-6;
    ROS_INFO_STREAM("bev tick :" << bev_image_tick);
    ROS_INFO_STREAM("publish birdview seq:" << bev_image_iter->Head.Seq);

    // stacked screen display
    auto raw_ins_pts_iter = rawInsPsDatasMap.find(bev_image_tick);

    if (raw_ins_pts_iter != rawInsPsDatasMap.end()) {
      std::uint64_t raw_ins_pts_tick = raw_ins_pts_iter->first;
      ROS_INFO_STREAM("raw_ins_pts tick:" << raw_ins_pts_tick);
      mviz1::data_visualization::DrawRawInsParkingSpace(bev_image_iter->data, raw_ins_pts_iter->second);

      // 清掉之前的
      raw_ins_pts_iter++;
      for (auto iter = rawInsPsDatasMap.begin(); iter != raw_ins_pts_iter;) {
        rawInsPsDatasMap.erase(iter++);
      }
      ROS_INFO_STREAM("raw_ins_pts_datas_map size:" << rawInsPsDatasMap.size());

      auto parkingspace_mviz_iter = parkingSpaceMvizBuf.find(raw_ins_pts_tick);

      if (parkingspace_mviz_iter != parkingSpaceMvizBuf.end()) {
        ROS_INFO_STREAM("parkingspace_mviz tick:" << parkingspace_mviz_iter->first);

        mviz1::data_visualization::DrawParkingSpace(bev_image_iter->data, parkingspace_mviz_iter->second);

        for (auto iter = parkingSpaceMvizBuf.begin(); iter != parkingspace_mviz_iter;) {
          parkingSpaceMvizBuf.erase(iter++);
        }
        ROS_INFO_STREAM("parkingspace_mviz_buf size:" << parkingSpaceMvizBuf.size());
      }
    }

    std_msgs::Header head;
    head.seq = imgSeq;
    head.stamp = bev_image_iter->timestamp;
    head.frame_id = "map";
    cv::Rect roi(0, 0, 352, 480);

    sensor_msgs::ImagePtr msgBv = cv_bridge::CvImage(head, "bgr8", bev_image_iter->data(roi)).toImageMsg();
    imageBevPub_.publish(msgBv);

    // parkingspace in odometry

    auto parkingspace_iter = parkingspaceDatasMap.find(bev_image_tick);

    if (parkingspace_iter != parkingspaceDatasMap.end()) {
      std::uint64_t parkingspace_tick = parkingspace_iter->first;
      ROS_DEBUG_STREAM("parkingspace_tick :" << parkingspace_tick);

      perception::ParkingSpace &parkingSpace = parkingspace_iter->second;
      if (managerMarker->UpdateParkingSpaceMarker(parkingSpace)) {
        parkingspaceVizPub.publish(managerMarker->parkingspace_markers_);
      }
      parkingspace_iter++;
      for (auto iter = parkingspaceDatasMap.begin(); iter != parkingspace_iter;) {
        parkingspaceDatasMap.erase(iter++);
      }
      ROS_INFO_STREAM("parkingspaceDatasMap size:" << parkingspaceDatasMap.size());

      // todo:: recommand slot attrubution and sected the slot attribution
      std::uint16_t planningToHmiSize = planningToHmiBuf.size();

      if (planningToHmiSize == 0) {
        ROS_WARN_STREAM("planning_to_hmi buff is empty");
        return 0;
      }
      for (auto index = 0; index < planningToHmiSize; index++) {
        minieye::PlanningToHMI &PlanningToHMI = planningToHmiBuf.front();
        int64_t tick_dt = planningToHmiBuf.front().tick() * 1e-3 - parkingspace_tick;
        if (abs(tick_dt) < sync_tick_dt_ms_) {
          ROS_INFO_STREAM("planningToHmi tick:" << planningToHmiBuf.front().tick() << "parkingspace_tick:"
                                                << parkingspace_tick << "sync tick_dt:" << tick_dt);
          managerMarker->UpdateSlotsRecommendAtt(PlanningToHMI);
          parkingspaceVizPub.publish(managerMarker->parkingspace_markers_);
          planningToHmiBuf.pop_front();
          break;
        } else if (tick_dt > sync_tick_dt_ms_) {
          break;
        } else {
          planningToHmiBuf.pop_front();
        }
      }

      std::uint16_t apaStateControlSize = APAStateControlBuf.size();
      if (apaStateControlSize == 0) {
        ROS_WARN_STREAM("apa_state_control buff is empty");
      }
      for (auto index = 0; index < apaStateControlSize; index++) {
        minieye::APAStateControl &apaStateControl = APAStateControlBuf.front();

        int64_t tick_dt = APAStateControlBuf.front().tick() * 1e-3 - parkingspace_tick;
        if (abs(tick_dt) < sync_tick_dt_ms_) {
          ROS_WARN_STREAM("apa_state_control tick :" << APAStateControlBuf.front().tick() << "parkingspace tick::"
                                                     << parkingspace_tick << " sync tick_dt:" << tick_dt);
          if (managerMarker->UpdateSlotsSelectedAtt(apaStateControl)) {
            parkingspaceVizPub.publish(managerMarker->parkingspace_markers_);
          }

          APAStateControlBuf.pop_front();
          break;
        } else if (tick_dt > sync_tick_dt_ms_) {
          break;
        } else {
          APAStateControlBuf.pop_front();
        }
      }
    }

    ROS_INFO_STREAM("bev image buffer size:" << imgFramesBev.size());
    return 1;
  } else {
    ROS_DEBUG_STREAM("bev image corresponding to fish image don not find ");
  }
  return 0;
}

int ApaData2Ros::Pub3DCoordinate(std::uint64_t &fish_image_tick) {
  // todo:缓存函数，topic 同步函数、更新定位函数
  // todo:task:缓存时间差阈值，同步时间差阈值，打印每个topic时间差，？确定本地发出的odometry能否全部收到

  size_t count = parkingOdometryMsgMap.size();

  for (size_t i = 0; i < count; i++) {
    minieye::Odometry3D &parkingOdometryMsg = parkingOdometryMsgMap.front();
    int64_t tick = parkingOdometryMsg.tick() * 1e-3;
    int64_t tickDt = static_cast<int64_t>(tick - fish_image_tick);
    ROS_INFO_STREAM("odometry_3d tick dt:" << tickDt);
    if (abs(tickDt) < sync_tick_dt_ms_) {
      ROS_INFO_STREAM("odometry_3d tick corresponding to the image:" << tick << ",image_fish_tick:" << fish_image_tick);

      if (parkingOdometryMsg.distance() < 0.5) {
        this->clearMarkerFlag = true;
      } else {
        this->clearMarkerFlag = false;
      }
      firstLocationFlag = false;

      tf::Transform transform;
      double x = parkingOdometryMsg.pose().x();
      double y = parkingOdometryMsg.pose().y();
      double z = parkingOdometryMsg.pose().z();
      double roll = parkingOdometryMsg.pose().roll();
      double pitch = parkingOdometryMsg.pose().pitch();
      double yaw = parkingOdometryMsg.pose().yaw();

      if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
        ROS_ERROR_STREAM("odometry3D point exist nan value");
        break;
      }

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

      parkingOdometryMsgMap.pop_front();
      break;
    } else if (tickDt > sync_tick_dt_ms_) {
      break;
    } else {
      parkingOdometryMsgMap.pop_front();
    }
  }

  count = planningDatasMap.size();
  for (size_t i = 0; i < count; i++) {
    minieye::Planning &planMsg = planningDatasMap.front();
    int64_t sync_tick_dt = static_cast<int64_t>(planningDatasMap.front().tick() * 1e-3 - fish_image_tick);

    if (abs(sync_tick_dt) < sync_tick_dt_ms_) {
      // 0-STANDBY， 1-ACTIVE， 2-SUSPEND， 3-REPLAN, 4-FINISHED, 5-FAILURE
      mviz1::data_conversion::PlanToRosMsg(planMsg, planning_);
      planning_to_qt_pub_.publish(planning_);

      if (planMsg.planning_status() == 1) {
        float width = carModel->m_width;
        std::uint64_t planningTick = planningDatasMap.front().tick() * 1e-3;

        if (managerMarker->UpdatePlanningMarker(planMsg, width)) {
          planingVizPub.publish(managerMarker->planning_markers_);
        }

        std::uint16_t planningToHmiSize = planningToHmiRvizBuf.size();

        for (auto index = 0; index < planningToHmiSize; index++) {
          minieye::PlanningToHMI &PlanningToHMI = planningToHmiRvizBuf.front();
          int64_t sync_tick_dt = planningToHmiRvizBuf.front().tick() * 1e-3 - planningTick;
          if (abs(sync_tick_dt) < sync_tick_dt_ms_) {
            if (managerMarker->UpdatePlanningState(PlanningToHMI.remain_distance(), PlanningToHMI.gear())) {
              planningStateVizPub.publish(managerMarker->planning_state_marker_);
            }

            planningToHmiRvizBuf.pop_front();
            break;
          } else if (sync_tick_dt > sync_tick_dt_ms_) {
            break;
          } else {
            planningToHmiRvizBuf.pop_front();
          }
        }
      }

      planningDatasMap.pop_front();
      break;
    } else if (sync_tick_dt > sync_tick_dt_ms_) {
      break;
    } else {
      planningDatasMap.pop_front();
    }
  }

  // todo:显示control中的两个字段 tick对齐

  count = controlDatasMap.size();
  for (size_t i = 0; i < count; i++) {
    minieye::VehicleControl &controlMsg = controlDatasMap.front();
    int64_t sync_tick_dt = static_cast<int64_t>(controlDatasMap.front().tick() * 1e-3 - fish_image_tick);

    if (abs(sync_tick_dt) < sync_tick_dt_ms_)  // 100ms
    {
      mviz1::data_conversion::ControlData2RosMsg(controlMsg, vehicleControl_);
      controlMsgPub.publish(vehicleControl_);
      controlDatasMap.pop_front();

      break;
    } else if (sync_tick_dt > sync_tick_dt_ms_) {
      break;
    } else {
      controlDatasMap.pop_front();
    }
  }

  count = parkingTargetDatasQue.size();
  for (size_t i = 0; i < count; i++) {
    minieye::parking::ObjectTrackList &pb = parkingTargetDatasQue.front();
    int64_t sync_tick_dt = static_cast<int64_t>(parkingTargetDatasQue.front().tick() * 1e-3 - fish_image_tick);

    if (abs(sync_tick_dt) < sync_tick_dt_ms_)  // 100ms
    {
      managerMarker->UpdateParkingTargetMarker(pb);
      parkingTargetPub.publish(managerMarker->parking_target_markers_);
      parkingTargetDatasQue.pop_front();

      break;
    } else if (sync_tick_dt > sync_tick_dt_ms_) {
      break;
    } else {
      parkingTargetDatasQue.pop_front();
    }
  }
  return 0;
}

bool ApaData2Ros::PubApmapResponseStatus(std::uint64_t &fish_image_tick) {
  const auto apmap_response_size = apmap_response_datas_quene_.size();

  for (size_t i = 0; i < apmap_response_size; i++) {
    minieye::parking::ApMapResponse &ap_map = apmap_response_datas_quene_.front();

    std::uint64_t ap_map_response_tick = apmap_response_datas_quene_.front().tick();
    int64_t sync_tick_dt = static_cast<int64_t>(ap_map_response_tick * 1e-3 - fish_image_tick);

    // 100ms
    if (abs(sync_tick_dt) < sync_tick_dt_ms_) {
      // 转ros_msg 发布出去
      mviz1::data_conversion::ApmapResponseToRosMsg(ap_map, apmap_response_);
      apmap_response_to_qt_pub_.publish(apmap_response_);

      std::cout << "==== ap_map tick:" << ap_map_response_tick << std::endl;

      // todo::找到对应的map_response;记录当前的转换矩阵给map_engine,用于后续的数据转换
      auto it = std::find_if(map_engine_response_datas_quene_.begin(), map_engine_response_datas_quene_.end(),
                             [&](minieye::parking::MapEngineResponse &map_engine) {
                                return ap_map_response_tick == static_cast<std::uint64_t>(map_engine.tick());
                             });

      if (it != map_engine_response_datas_quene_.end()) {
       
        auto pose = ap_map.ap_enu_to_rt_odom();
        Eigen::Isometry3d enu_to_odom = Eigen::Isometry3d::Identity();
        enu_to_odom.translation() = Eigen::Vector3d(pose.x(), pose.y(), pose.z());

        // Eigen:wxyz
        Eigen::Matrix3d rotation_matrix =
            Eigen::Quaterniond(pose.qw(), pose.qx(), pose.qy(), pose.qz()).normalized().toRotationMatrix();

        enu_to_odom.linear() = rotation_matrix;

        minieye::parking::MapEngineResponse &map_engine = *it;
        if (map_engine_marker_->Update(enu_to_odom, map_engine)) {
          map_engine_response_pub_.publish(map_engine_marker_->GetMarkerArray());
        }

      } else {
        ROS_INFO_STREAM("current ap_map does not corespond to map_engine:");
      }
      apmap_response_datas_quene_.pop_front();

      break;
    } else if (sync_tick_dt > sync_tick_dt_ms_) {
      break;
    } else {
      apmap_response_datas_quene_.pop_front();
    }
  }
  return 0;
}

bool ApaData2Ros::PublishUssParkingSpace(std::uint64_t &current_fishimage_tick) {
  // todo::sync uss_parkingspace and image
  for (size_t index = 0; index < uss_parkingspace_mviz_buffer_.size(); index++) {
    perception::ParkingSpace &ussParkingSpaceMsg = uss_parkingspace_mviz_buffer_.front();
    std::uint64_t uss_parkingspace_tick = ussParkingSpaceMsg.tick() * 1e-3;

    int64_t sync_tick_dt_ms = static_cast<int64_t>(uss_parkingspace_tick - current_fishimage_tick);
    ROS_INFO_STREAM("buffer uss_parkingspace_tick dt:" << sync_tick_dt_ms);
    if (abs(sync_tick_dt_ms) < sync_tick_dt_ms_) {
      ROS_INFO_STREAM("uss_parkingspace_tick:" << uss_parkingspace_tick << ",corespondent_fishimage_tick "
                                               << current_fishimage_tick << ",sync_tick_dt_ms:" << sync_tick_dt_ms);

      if (managerMarker->UpdateUssParkingspaceMarker(ussParkingSpaceMsg)) {
        uss_parkingspace_publish.publish(managerMarker->uss_parkingspace_markers_);
      }
      uss_parkingspace_mviz_buffer_.pop_front();
    } else if (sync_tick_dt_ms > sync_tick_dt_ms_) {
      break;
    } else {
      uss_parkingspace_mviz_buffer_.pop_front();
    }
  }
  return true;
}

void ApaData2Ros::LoadLaunchPara() {
  ROS_INFO_STREAM(__FUNCTION__ << ",start");
  private_nh.param<std::string>("/config_dir", mviz_config_dir_,
                                "/home/nico/minieye/mviz/mviz2_ros/src/"
                                "mviz/config/apa_hil_board_config.json");
  private_nh.param<double>("/ros_rate", pubRate, 20);
  private_nh.param<double>("/car_model_size", carModelSize, 1.0);

  private_nh.param<bool>("/use_hardware_decode", useHwdecode, true);

  int cacheImageNumbertmp;
  private_nh.param<int>("/number_image_cache", cacheImageNumbertmp, 20);
  cacheImageNumber = static_cast<size_t>(cacheImageNumbertmp);

  private_nh.param<int>("/cache_tick_dt_ms", cache_tick_dt_ms_, 100);
  private_nh.param<int>("/sync_tick_dt_ms", sync_tick_dt_ms_, 100);

  private_nh.param<std::string>("/odometry_type", m_odometry_type, "3D");
  private_nh.param<int>("/show_wait_time", show_wait_time_, 300);

  private_nh.param<int>("/image_front_position", image_front_position, 1);
  private_nh.param<int>("/image_rear_position", image_rear_position, 2);
  private_nh.param<int>("/image_left_position", image_left_position, 3);
  private_nh.param<int>("/image_right_position", image_right_position, 4);

  private_nh.param<std::string>("/mviz_mode", mviz_replay_mode_, "hil");

  private_nh.param<bool>("/show_uss_parkingspace", bShowUssParkingspace_, true);
  private_nh.param<std::string>("/car_body_dir", carBodyDir, "src/mviz/config/x50_body.yaml");
  private_nh.param<float>("/marker_scale", markerScale, 0.3);
  private_nh.param<float>("/marker_radio", markerRadio, 1.0);
  private_nh.param<float>("/maker_lifetime", maker_lifetime_, 650.f);  // in ms

  ROS_INFO_STREAM(__FUNCTION__ << ",finish");
}

void ApaData2Ros::InitPublish() {
  ROS_INFO_STREAM(__FUNCTION__ << ",start");
  // publish rosmsg for rviz visualization

  path.header.frame_id = "odome";
  path.header.stamp = ros::Time::now();

  imagePubLeftUp_ = it_.advertise("fish_camera1", 10);
  imagePubLeftDown_ = it_.advertise("fish_camera2", 10);
  imagePubRightUp_ = it_.advertise("fish_camera3", 10);
  imagePubRightDown_ = it_.advertise("fish_camera4", 10);
  imageBevPub_ = it_.advertise("bird_view_img", 10);

  clearMakerPub = nh_.advertise<visualization_msgs::MarkerArray>("/viz/text", 10);
  carMarkerPub = nh_.advertise<visualization_msgs::MarkerArray>("/viz/car_model2D", 10);

  parkingOdometryVizPub = nh_.advertise<nav_msgs::Path>("/viz/odometry_3d", 10);

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

  planning_to_qt_pub_ = nh_.advertise<mviz_apa_show::Planning>("/qt_msg/planning", 10);
  apmap_response_to_qt_pub_ = nh_.advertise<mviz_apa_show::ApMapResponse>("/qt_msg/apmap_response", 10);
  map_engine_response_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/vis/map_engine_response", 10);
  ROS_INFO_STREAM(__FUNCTION__ << ",finish");
}

// hardware video decode
bool ApaData2Ros::CacheImgData(std::string topic, std::deque<ImageFlowHeaderStruct> &imgHeader,
                               std::deque<ImageFlowStruct> &imgFrames, decode_video_hw *imgDecoder) {
  // Cache fish img buf and decode to cv::Mat
  bool ret = false;
  collect::Recv_Topic_Data_ST recvData;
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    ret = true;
    ros::Time timestamp = ros::Time::now();
    ImageFlowHeaderStruct header = *reinterpret_cast<ImageFlowHeaderStruct *>(recvData.buff);
    imgHeader.push_back(header);

    const size_t yuvSize = size_t(header.Height * header.Width * 2.0);
    uint8_t *outbuf = (uint8_t *)malloc(yuvSize);
    int grap = 0;
    int status = 0;
    int w, h;
    imgDecoder->parse(reinterpret_cast<uint8_t *>(recvData.buff + 36), header.DataSize);

    imgDecoder->decode(&w, &h, &status, &grap);

    AVFrame frame = imgDecoder->get_frame();
    if (frame.width == 0 || frame.height == 0) {
      ROS_ERROR("hardware decode failed ,status: %d !", status);
      free(outbuf);
      continue;
    }
    AVPixelFormat fmt = (AVPixelFormat)frame.format;
    int buf_size = av_image_get_buffer_size(fmt, w, h, 1);
    av_image_copy_to_buffer(outbuf, buf_size, frame.data, frame.linesize, fmt, w, h, 1);

    // yuv 2 bgr
    cv::Mat bgr;
    cv::Mat yuv(cv::Size(header.Width, int(header.Height * 1.5)), CV_8UC1, outbuf);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
    if (bgr.empty()) {
      ROS_ERROR("cvtColor: img is empty!");
      free(outbuf);
      continue;
    }

    ImageFlowStruct imgFrameTemp;
    imgFrameTemp.topic = topic;
    imgFrameTemp.timestamp = timestamp;
    imgFrameTemp.Head = imgHeader.front();
    imgFrameTemp.data = bgr;

    imgFrames.push_back(imgFrameTemp);
    imgHeader.pop_front();

    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
    free(outbuf);

    if (!imgFramesFish.empty()) {
      if (imgFrameTemp.Head.Seq > imgFramesFish.front().Head.Seq + 10 + cacheImageNumber) {
        return ret;
      }
    }
  }
  return ret;
}

// software video decode
bool ApaData2Ros::CacheImgData(std::string topic, std::deque<ImageFlowHeaderStruct> &imgHeader,
                               std::deque<ImageFlowStruct> &imgFrames, decode_video_sw *imgDecoder) {
  // Cache fish img buf and decode to cv::Mat
  bool ret = false;
  collect::Recv_Topic_Data_ST recvData;
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    ret = true;
    ros::Time timestamp = ros::Time::now();
    ImageFlowHeaderStruct header = *reinterpret_cast<ImageFlowHeaderStruct *>(recvData.buff);
    // PrintHead(header);
    // std::cout << topic <<" sss1:" << *(reinterpret_cast<const uint32_t
    // *>(recvData.buff + 28)) << std::endl; std::cout << topic <<" sss2:" <<
    // *(reinterpret_cast<const uint32_t *>(recvData.buff + 32)) << std::endl;
    imgHeader.push_back(header);

    const size_t yuvSize = size_t(header.Height * header.Width * 2.0);
    uint8_t *outbuf = (uint8_t *)malloc(yuvSize);
    int status = 0;
    int grap = 0;

    imgDecoder->decode(reinterpret_cast<uint8_t *>(recvData.buff + 36), header.DataSize, outbuf, &status, &grap);
    if (status != 1) {
      ROS_ERROR("decode failed ,status: %d !", status);
      free(outbuf);
      return false;
    }
    // yuv 2 bgr
    cv::Mat bgr;
    cv::Mat yuv(cv::Size(header.Width, int(header.Height * 1.5)), CV_8UC1, outbuf);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
    if (bgr.empty()) {
      ROS_ERROR("cvtColor: img is empty!");
      free(outbuf);
      continue;
    }

    ImageFlowStruct imgFrameTemp;
    imgFrameTemp.topic = topic;
    imgFrameTemp.timestamp = timestamp;
    imgFrameTemp.Head = imgHeader.front();
    imgFrameTemp.data = bgr;

    ROS_INFO_STREAM("imgFrames.push_back " << imgFrameTemp.Head.Seq);
    imgFrames.push_back(imgFrameTemp);
    imgHeader.pop_front();

    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
    free(outbuf);

    if (!imgFramesFish.empty()) {
      if (imgFrameTemp.Head.Seq > imgFramesFish.front().Head.Seq + 10 + cacheImageNumber) {
        return ret;
      }
    }
  }
  return ret;
}

bool ApaData2Ros::CacheImgData() {
  bool ret = false;
  std::string topic;
  if (mviz_replay_mode_ == "oncar") {
    topic = "camera_stitching";
  } else {
    topic = "camera30";
  }
  if (imgDecoderFishHW != nullptr) {
    ret = CacheImgData(topic, imgFishHeader, imgFramesFish, imgDecoderFishHW);
  } else {
    ret = CacheImgData(topic, imgFishHeader, imgFramesFish, imgDecoderFishSW);
  }
  // bev_libflow
  if (imgDecoderBevHW != nullptr) {
    CacheImgData(bev_topic, imgBirdviewHeader, imgFramesBev, imgDecoderBevHW);
  } else {
    CacheImgData(bev_topic, imgBirdviewHeader, imgFramesBev, imgDecoderBevSW);
  }
  return ret;
}

void ApaData2Ros::CacheFlowPbData() {
  std::string topic;
  collect::Recv_Topic_Data_ST recvData;
  topic = "vehicle_signal";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::VehicleSignal vehicle_signal;
    vehicle_signal.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());
    //@TODO vehicle_signal没有做对齐就发出去了,
    mviz1::data_conversion::VehicleSignal2RosMsg(vehicle_signal, vehicleSignal_);
    vehicleSignalMsgPub.publish(vehicleSignal_);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  // topic = "odo_vehicle_signal";
  // while (!_bWantStop && minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer",
  // recvData)) {
  //   auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
  //   auto obj = handle.get();                                      // 得到反序列化对象
  //   obj.convert(srcMsg_);
  //   // protobuf 反序列化
  //   minieye::OdoVehicleSignal Odo_vehicle_signal;
  //   Odo_vehicle_signal.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

  //   // 用完释放buff
  //   if (recvData.buff != nullptr) {
  //     delete recvData.buff;
  //     recvData.buff = nullptr;
  //   }
  // }

  // topic = "odo_vehicle_signal";
  // while (!_bWantStop && minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer",
  // recvData)) {
  //   auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
  //   auto obj = handle.get();                                      // 得到反序列化对象
  //   obj.convert(srcMsg_);
  //   // protobuf 反序列化
  //   minieye::OdoVehicleSignal Odo_vehicle_signal;
  //   Odo_vehicle_signal.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

  //   // 用完释放buff
  //   if (recvData.buff != nullptr) {
  //     delete recvData.buff;
  //     recvData.buff = nullptr;
  //   }
  // }

  // topic = "apa_vehicle_signal";
  // while (!_bWantStop && minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer",
  // recvData)) {
  //   auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
  //   auto obj = handle.get();                                      // 得到反序列化对象
  //   obj.convert(srcMsg_);
  //   // protobuf 反序列化
  //   minieye::ApaVehicleSignal apa_vehicle_signal;
  //   apa_vehicle_signal.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

  //   // 用完释放buff
  //   if (recvData.buff != nullptr) {
  //     delete recvData.buff;
  //     recvData.buff = nullptr;
  //   }
  // }

  // topic = "imu";
  // while (!_bWantStop && minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer",
  // recvData)) {
  //   auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
  //   auto obj = handle.get();                                      // 得到反序列化对象
  //   obj.convert(srcMsg_);
  //   // protobuf 反序列化
  //   minieye::ImuDataList imu;
  //   imu.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

  //   // 用完释放buff
  //   if (recvData.buff != nullptr) {
  //     delete recvData.buff;
  //     recvData.buff = nullptr;
  //   }
  // }

  // topic = "apa_gnss";
  // while (!_bWantStop && minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer",
  // recvData)) {
  //   auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
  //   auto obj = handle.get();                                      // 得到反序列化对象
  //   obj.convert(srcMsg_);
  //   // protobuf 反序列化
  //   minieye::GnssData apa_gnss;
  //   apa_gnss.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

  //   // 用完释放buff
  //   if (recvData.buff != nullptr) {
  //     delete recvData.buff;
  //     recvData.buff = nullptr;
  //   }
  // }

  topic = "uss";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::UltrasonicRadar uss_radar;
    uss_radar.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());
    mviz1::data_conversion::UssRadar2RosMsg(uss_radar, uss_radar_);
    uss_radar_publish_.publish(uss_radar_);

    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  // topic = "apa_state_control";
  // while (!_bWantStop && minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer",
  // recvData)) {
  //   auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
  //   auto obj = handle.get();                                      // 得到反序列化对象
  //   obj.convert(srcMsg_);
  //   // protobuf 反序列化
  //   minieye::APAStateControl state_control;
  //   state_control.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

  //   APAStateControlBuf.push_back(state_control);

  //   // 用完释放buff
  //   if (recvData.buff != nullptr) {
  //     delete recvData.buff;
  //     recvData.buff = nullptr;
  //   }
  // }

  topic = "odometry_3d";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::Odometry3D odometry_3d;
    odometry_3d.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    parkingOdometryMsgMap.push_back(odometry_3d);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "raw_ins_ps";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    perception::RawInsParkingSpace raw_ins_pts;
    raw_ins_pts.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    uint64_t tick = raw_ins_pts.tick() * 1e-3;
    rawInsPsDatasMap[tick] = raw_ins_pts;
    ROS_INFO_STREAM("[" << topic << "] pushed, tick" << tick);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  // template <typename T>
  // void Getdata(){

  // };
  // Getdata<perception::RawInsParkingSpace>();

  topic = "freespace";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    freespacepoints::FreespacePoints freespace_points;
    freespace_points.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    int seqtemp = freespace_points.frame_id();
    freespaceDatasMap[seqtemp] = freespace_points;
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "gridmap";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    perception::FreespaceObstacles freespace_obstacles;
    freespace_obstacles.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    uint64_t tick = freespace_obstacles.tick();
    gridmapDatasMap[tick] = freespace_obstacles;
    ROS_DEBUG_STREAM("grimap  tick :" << tick);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "parkingspace";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    perception::ParkingSpace parkingspace;
    parkingspace.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    std::uint64_t tick = parkingspace.tick() * 1e-3;
    parkingspaceDatasMap[tick] = parkingspace;
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "parkingspace_mviz";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    perception::ParkingSpace parkingspace;
    parkingspace.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    std::uint64_t tick = parkingspace.tick() * 1e-3;
    parkingSpaceMvizBuf[tick] = parkingspace;
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "uss_parkingspace_mviz";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    perception::ParkingSpace uss_parkingspace_mviz;
    uss_parkingspace_mviz.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    uss_parkingspace_mviz_buffer_.push_back(uss_parkingspace_mviz);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "planning";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::Planning planning;
    planning.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    planningDatasMap.push_back(planning);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "planning_to_hmi";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::PlanningToHMI planning_to_hmi;
    planning_to_hmi.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    ROS_INFO_STREAM("planning_to_hmi cache tick:" << static_cast<std::uint64_t>(planning_to_hmi.tick()));
    minieye::PlanningToHMI planning_to_hmi_rviz;
    planning_to_hmi_rviz.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    planningToHmiBuf.push_back(planning_to_hmi);
    planningToHmiRvizBuf.push_back(planning_to_hmi_rviz);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "vehicle_control";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::VehicleControl vehicle_control;
    vehicle_control.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    controlDatasMap.push_back(vehicle_control);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  topic = "parking_target";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::parking::ObjectTrackList pb;
    pb.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());

    parkingTargetDatasQue.push_back(pb);
    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }
  topic = "ap_map_response";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::parking::ApMapResponse pb;
    pb.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());
    apmap_response_datas_quene_.push_back(pb);

    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  //拿map_engine_reponse 数据
  topic = "map_engine_response";
  while (!_bWantStop &&
         minieye::mviz::collect::CDataCollectDBMan::get_instance()->get_topic_data(topic + "_viewer", recvData)) {
    auto handle = msgpack::unpack(recvData.buff, recvData.size);  // 输入二进制数据
    auto obj = handle.get();                                      // 得到反序列化对象
    obj.convert(srcMsg_);
    // protobuf 反序列化
    minieye::parking::MapEngineResponse pb;
    pb.ParseFromArray(srcMsg_.data.data(), srcMsg_.data.size());
    map_engine_response_datas_quene_.push_back(pb);

    // 用完释放buff
    if (recvData.buff != nullptr) {
      delete recvData.buff;
      recvData.buff = nullptr;
    }
  }

  return;
}

void ApaData2Ros::ClearCacheFlowPbData() {
  // ROS_WARN_STREAM("----clear cache----" );
  // ROS_WARN_STREAM("imgFramesFish size:" << imgFramesFish.size());
  // ROS_WARN_STREAM("imgFramesBev size:" << imgFramesBev.size());
  // ROS_WARN_STREAM("parkingspaceDatasMap size:" <<
  // parkingspaceDatasMap.size()); ROS_WARN_STREAM("freespaceDatasMap size:" <<
  // freespaceDatasMap.size()); ROS_WARN_STREAM("rawInsPsDatasMap size:" <<
  // rawInsPsDatasMap.size()); ROS_WARN_STREAM("gridmapDatasMap size:" <<
  // gridmapDatasMap.size()); ROS_WARN_STREAM("parkingSpaceMvizBuf size:" <<
  // parkingSpaceMvizBuf.size()); ROS_WARN_STREAM("parkingOdometryMsgMap size:"
  // << parkingOdometryMsgMap.size()); ROS_WARN_STREAM("planningDatasMap size:"
  // << planningDatasMap.size()); ROS_WARN_STREAM("controlDatasMap size:" <<
  // controlDatasMap.size()); ROS_WARN_STREAM("APAStateControlBuf size:" <<
  // APAStateControlBuf.size()); ROS_WARN_STREAM("planningToHmiBuf size:" <<
  // planningToHmiBuf.size()); ROS_WARN_STREAM("planningToHmiRvizBuf size:" <<
  // planningToHmiRvizBuf.size()); ROS_WARN_STREAM("uss_parkingspace_buffer
  // size:" << uss_parkingspace_buffer.size());

  imgFramesFish.clear();
  imgFramesBev.clear();
  parkingspaceDatasMap.clear();
  freespaceDatasMap.clear();
  rawInsPsDatasMap.clear();
  gridmapDatasMap.clear();
  parkingSpaceMvizBuf.clear();
  parkingOdometryMsgMap.clear();
  planningDatasMap.clear();
  controlDatasMap.clear();
  APAStateControlBuf.clear();
  planningToHmiBuf.clear();
  planningToHmiRvizBuf.clear();
  uss_parkingspace_mviz_buffer_.clear();
  parkingTargetDatasQue.clear();
  apmap_response_datas_quene_.clear();
  map_engine_response_datas_quene_.clear();
  // ROS_WARN_STREAM("----clear cache end----" );
  return;
}

void ApaData2Ros::PublishFlowData() {
  ros::Rate rate(pubRate);
  ImageFlowStruct imgFrametemp;
  std::uint64_t cur_fish_image_tick;
  uint32_t cur_fish_image_seq;
  bool init_time = false;

  while (nh_.ok() && !_bWantStop) {
    rate.sleep();
    // ROS_INFO_STREAM("imgFramesFish.size " << imgFramesFish.size());
    // ROS_INFO_STREAM("imgFramesBev.size " << imgFramesBev.size());

    auto received_fish_image_data = CacheImgData();
    if (received_fish_image_data) {
      init_time = true;
    } else if (!received_fish_image_data && !init_time) {
      ROS_INFO_STREAM("mviz display thread does_not receive fish image data!");
      continue;
    }
    CacheFlowPbData();
    // todo::wait for apa_module_topic_data ,especially for using HiL-SIM-MODE
    // todo::show buffer left data when receive thread has not receive fish
    // image data
    if (imgFramesFish.size() < cacheImageNumber && received_fish_image_data) {
      continue;
    }

    if (imgFramesFish.size() == 0) {
      ROS_INFO_STREAM("all datas have been showed finishly");
      ClearCacheFlowPbData();
      continue;
    }

    ImageFlowStruct cur_fish_image_frame = imgFramesFish.front();
    cur_fish_image_seq = cur_fish_image_frame.Head.Seq;
    cur_fish_image_tick = cur_fish_image_frame.Head.Sec * 1e3 + cur_fish_image_frame.Head.Nsec * 1e-6;

    ROS_INFO_STREAM("current fish image buffer tick:" << cur_fish_image_tick);
    ROS_INFO_STREAM("current fish image buffer seq:" << cur_fish_image_seq);
    // ROS_INFO_STREAM("fish image buffer size " << imgFramesFish.size());

    PubFishImgAndFreespace();
    PubBirdviewImg(cur_fish_image_seq, cur_fish_image_tick);
    Pub3DCoordinate(cur_fish_image_tick);
    PubApmapResponseStatus(cur_fish_image_tick);

    // PubMapEngineResponse(cur_fish_image_tick);

    if (bShowUssParkingspace_) {
      PublishUssParkingSpace(cur_fish_image_tick);
    }

    if (this->clearMarkerFlag) {
      clearMarker("odome");
      clearMarker("base_link");
    }

    ros::spinOnce();
  }
  return;
}

void ApaData2Ros::start() {
  ROS_INFO_STREAM(__FUNCTION__ << __LINE__);
  pub_thread = new std::thread(&ApaData2Ros::PublishFlowData, this);
  return;
}

void ApaData2Ros::stop() {
  ROS_INFO_STREAM(__FUNCTION__ << __LINE__);
  _bWantStop = true;
}

int ApaData2Ros::init() {
  ROS_INFO_STREAM(__FUNCTION__ << __LINE__);
  LoadLaunchPara();
  InitMarker();
  InitPublish();
  return 1;
}

void ApaData2Ros::InitMarker() {
  carModel = std::make_shared<GenerateCarModel>(carBodyDir, markerRadio, markerScale);
  carModel->MakeCarModel(carMarker);
  managerMarker = std::make_shared<ManagerMarker>(markerScale);
  managerMarker->SetMarkerLifetime(maker_lifetime_);
  map_engine_marker_ = std::make_shared<MapEngineMarker>(markerScale,carModel->m_width);
}

ApaData2Ros::ApaData2Ros() : ApaData2Ros(AV_CODEC_ID_H264) { ROS_INFO_STREAM(__FUNCTION__ << __LINE__); }

ApaData2Ros::ApaData2Ros(int codetype = AV_CODEC_ID_H264) : codeType(codetype), it_(nh_), private_nh("~") {
  ROS_INFO_STREAM(__FUNCTION__ << __LINE__);

  // try
  // {
  //     imgDecoderFishHW = new decode_video_hw(codeType);
  // }
  // catch(char const* error)
  // {
  //     ROS_DEBUG_STREAM("Init Qsv Fail! "
  //                       << "error: " << error
  //                       << ", Use CPU" );
  //     imgDecoderFishHW = nullptr;
  //     imgDecoderFishSW = new decode_video_sw(codeType);
  // }
  // try
  // {
  //     imgDecoderBevHW = new decode_video_hw(codeType);
  // }
  // catch(char const* error)
  // {
  //     ROS_DEBUG_STREAM("Init Qsv Fail! "
  //                       << "error: " << error
  //                       << ", Use CPU" );
  //     imgDecoderBevHW = nullptr;
  //     imgDecoderBevSW = new decode_video_sw(codeType);
  // }
  if (useHwdecode) {
    imgDecoderFishHW = new decode_video_hw(codeType);
    imgDecoderBevHW = new decode_video_hw(codeType);
  } else {
    imgDecoderFishSW = new decode_video_sw(codeType);
    imgDecoderBevSW = new decode_video_sw(codeType);
  }
  ROS_INFO_STREAM(__FUNCTION__ << __LINE__ << ", finish");
}

ApaData2Ros::~ApaData2Ros() {
  ROS_INFO_STREAM(__FUNCTION__ << __LINE__);
  if (pub_thread != nullptr) {
    if (pub_thread->joinable()) {
      pub_thread->join();
    }
    delete pub_thread;
    pub_thread = nullptr;
  }

  if (imgDecoderFishSW) {
    delete imgDecoderFishSW;
    imgDecoderFishSW = nullptr;
  }
  if (imgDecoderFishHW) {
    delete imgDecoderFishHW;
    imgDecoderFishHW = nullptr;
  }
  if (imgDecoderBevSW) {
    delete imgDecoderBevSW;
    imgDecoderBevSW = nullptr;
  }
  if (imgDecoderBevHW) {
    delete imgDecoderBevHW;
    imgDecoderBevHW = nullptr;
  }
}
