#include "display/viz_struct.hpp"

using namespace std;

std::map<int, std::string> VizLineType = {
    {0, "Solid_"},         // 实线
    {1, "Dashed_"},        // 虚线
    {2, "Bold_"},          // 短粗虚线
    {3, "DoubleSolid_"},   // 双实线
    {4, "DoubleDashed_"},  // 双虚线
    {5, "SolidDashed_"},   // 实虚线
    {6, "DashedSolid_"},   // 虚实线
    {7, "Fence_"},         // 栅栏
    {8, "Curb_"},          // 路沿
    {9, "Deceleration_"},  // 减速线
    {10, " "},
};
std::map<int, std::string> VizColorType = {
    {0, "White_"},       {1, "Yellow_"},      {2, "Orange_"},           {3, "Blue_"}, {4, "Green_"}, {5, "Gray_"},
    {6, "WhiteYellow_"}, {7, "YellowWhite_"}, {8, "YellowGrayFusion_"}, {9, " "},
};

std::map<int, std::string> VizLanelinePositionType = {
    {0, "LeftLeft_"}, {1, "Left_"}, {2, "Right_"}, {3, "RightRight_"}, {4, " "}, {5, " "}, {6, " "}, {7, " "}, {8, " "},
};

void SplitImg(cv::Mat& srcImg, int imgNum, std::vector<cv::Mat>& imgSplited) {
  imgSplited.clear();
  int imgWidth = srcImg.cols;
  int imgHeight = srcImg.rows;
  if (2 == imgNum) {
    cv::Rect leftImgRoi(0, 0, int(imgWidth / 2), imgHeight);
    cv::Rect rightImgRoi(int(imgWidth / 2), 0, int(imgWidth / 2), imgHeight);
    imgSplited.push_back(srcImg(leftImgRoi));
    imgSplited.push_back(srcImg(rightImgRoi));
  } else if (4 == imgNum) {
    cv::Rect imgRoiLeftUp(0, 0, int(imgWidth / 2), imgHeight / 2);
    cv::Rect imgRoiLeftDown(0, imgHeight / 2, int(imgWidth / 2), imgHeight / 2);
    cv::Rect imgRoiRightUp(int(imgWidth / 2), 0, int(imgWidth / 2), imgHeight / 2);
    cv::Rect imgRoiRightDown(int(imgWidth / 2), imgHeight / 2, int(imgWidth / 2), imgHeight / 2);
    imgSplited.push_back(srcImg(imgRoiLeftUp));
    imgSplited.push_back(srcImg(imgRoiLeftDown));
    imgSplited.push_back(srcImg(imgRoiRightUp));
    imgSplited.push_back(srcImg(imgRoiRightDown));
  } else {
    imgSplited.push_back(srcImg);
  }
}

std::vector<double> LinSpace(const double& x_start, const double& x_end, const int& num) {
  int x_size = num + 1;
  const auto space = (x_end - x_start) / num;
  std::vector<double> x(x_size);
  x[0] = x_start;

  for (int ii = 1; ii < x_size; ii++) {
    x[ii] = x[ii - 1] + space;
  }

  return x;
}

/**
 * @brief  根据车辆坐标与规划指令，输出规划轨迹采样
 * @param  x_start: 车辆坐标X(m)
 * @param  y_start: 车辆坐标Y(m)
 * @param  theta_start: 车辆朝向(rad)
 * @param  move_direct: 规划指令->前进后退(1/-1/0)
 * @param  trun_radius: 规划指令->转向半径(m)
 * @param  move_distance: 规划指令->行驶距离(m)
 * @param  ds_set: 采样点距(m)
 * @return 采样点序列std::vector<std::tuple<x,y,theta>>，单位同上面一致
 * @author 张鹏
 * @date   20220530
 */
std::vector<std::tuple<double, double, double>> PathSample(const double x_start, const double y_start,
                                                           const double theta_start, const int move_direct,
                                                           const double trun_radius, const double move_distance,
                                                           const double ds_set) {
  std::vector<std::tuple<double, double, double>> path_sample;
  if (move_distance < 0.01) {
    return path_sample;
  }
  const int sample_num = (std::max)({static_cast<long>(3), lround(move_distance / ds_set + 1)});  // 最少采样3个点
  auto s_list = LinSpace(0.0, move_distance, sample_num);                                         // 对轨迹采样

  // Planning path <0|invalid 1|R 2|D; turn_radius,positive|right,negetive|left; ds>
  if (move_direct == 1)  // 后退
  {
    for (auto& s : s_list) {
      s *= -1;
    }
  }
  if (trun_radius == 0.0)  // 直线
  {
    for (const auto& s : s_list) {
      double x = x_start + s * cos(theta_start);
      double y = y_start + s * sin(theta_start);
      double theta = theta_start;
      path_sample.emplace_back(std::make_tuple(x, y, theta));
    }
  } else {
    auto path_R_new = -trun_radius;  //注意源代码中将右转定为正，新的规则将左转定为正
    // 计算圆心
    auto xo = x_start - sin(theta_start) * path_R_new;
    auto yo = y_start + cos(theta_start) * path_R_new;
    for (const auto& s : s_list) {
      double theta = theta_start + s / path_R_new;
      auto theta_new = theta + 1.5 * 3.14159265358979;
      double x = xo + path_R_new * cos(theta_new);
      double y = yo + path_R_new * sin(theta_new);
      path_sample.emplace_back(std::make_tuple(x, y, theta));
    }
  }

  return path_sample;
}
//原文链接：https://blog.csdn.net/gs1069405343/article/details/83414131
// 图像、圆心、开始点、结束点、线宽
void DrawArc(cv::Mat* src, cv::Point ArcCenter, cv::Point StartPoint, cv::Point EndPoint, int Fill) {
  if (Fill <= 0) return;

  std::vector<cv::Point> Dots;
  double Angle1 = atan2((StartPoint.y - ArcCenter.y), (StartPoint.x - ArcCenter.x));
  double Angle2 = atan2((EndPoint.y - ArcCenter.y), (EndPoint.x - ArcCenter.x));
  double Angle = Angle1 - Angle2;
  Angle = Angle * 180.0 / CV_PI;

  if (Angle < 0) Angle = 360 + Angle;
  if (Angle == 0) Angle = 360;
  int brim = floor(Angle / 10);  // 向下取整

  Dots.push_back(StartPoint);
  for (int i = 0; i < brim; i++) {
    double dSinRot = sin(-(10 * (i + 1)) * CV_PI / 180);
    double dCosRot = cos(-(10 * (i + 1)) * CV_PI / 180);
    int x = ArcCenter.x + dCosRot * (StartPoint.x - ArcCenter.x) - dSinRot * (StartPoint.y - ArcCenter.y);
    int y = ArcCenter.y + dSinRot * (StartPoint.x - ArcCenter.x) + dCosRot * (StartPoint.y - ArcCenter.y);
    Dots.push_back(cv::Point(x, y));
  }
  Dots.push_back(EndPoint);
  cv::RNG& rng = cv::theRNG();
  cv::Scalar color = cv::Scalar(rng.uniform(100, 255), rng.uniform(100, 255), rng.uniform(100, 255));
  for (size_t i = 0; i < Dots.size() - 1; i++) {
    line(*src, Dots[i], Dots[i + 1], color, Fill);
  }
  Dots.clear();
}
/*
 * @brief  根据规划指令，输出车身坐标系规划轨迹maker
 * @param  moveDirect: 规划指令->前进后退(1/2)
 * @param  turnRadius: 规划指令->转向半径(m) >0右拐 =0直线 <0左拐
 * @param  moveDistance: 规划指令->行驶距离(m)
 * @param  marker：车身坐标系规划轨迹maker
 * @author 许瑞龙
 * @date   20220615
 */
void getPathPoints(size_t moveDirect, float turnRadius, float moveDistance, visualization_msgs::Marker& marker) {
  //直线
  if (turnRadius == 0) {
    for (int x = 0; x < moveDistance; x++) {
      if (moveDirect == 1)  //后退
      {
        geometry_msgs::Point pt;
        pt.x = -x;
        pt.y = 0;
        pt.z = 0;
        marker.points.push_back(pt);
        // marker.points.emplace_back(-x, 0, 0);
      }
      if (moveDirect == 2)  //前进
      {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = 0;
        pt.z = 0;
        marker.points.push_back(pt);
        // marker.points.emplace_back(x, 0, 0);
      }
    }
  }
  //圆弧
  else {
    float theta = moveDistance / abs(turnRadius);
    for (float th = 0; th < theta; th += theta / 10) {
      if (turnRadius < 0)  //左拐
      {
        if (moveDirect == 1)  //后退
        {
          float x = abs(turnRadius) * sin(th);
          float y = abs(turnRadius) * (1 - cos(th));
          geometry_msgs::Point pt;
          pt.x = -x;
          pt.y = y;
          pt.z = 0;
          marker.points.push_back(pt);
          // marker.points.emplace_back(-x, y, 0);
        }
        if (moveDirect == 2)  //前进
        {
          float x = abs(turnRadius) * sin(th);
          float y = abs(turnRadius) * (1 - cos(th));
          geometry_msgs::Point pt;
          pt.x = x;
          pt.y = y;
          pt.z = 0;
          marker.points.push_back(pt);
          // marker.points.emplace_back(x, y, 0);
        }
      } else  //右拐
      {
        if (moveDirect == 1)  //后退
        {
          float x = abs(turnRadius) * sin(th);
          float y = abs(turnRadius) * (1 - cos(th));
          geometry_msgs::Point pt;
          pt.x = -x;
          pt.y = -y;
          pt.z = 0;
          marker.points.push_back(pt);
          // marker.points.emplace_back(-x, -y, 0);
        }
        if (moveDirect == 2)  //前进
        {
          float x = abs(turnRadius) * sin(th);
          float y = abs(turnRadius) * (1 - cos(th));
          geometry_msgs::Point pt;
          pt.x = x;
          pt.y = -y;
          pt.z = 0;
          marker.points.push_back(pt);
        }
      }
    }
  }
}

visualization_msgs::Marker getcarModelMaker(std::string frameId, float scale) {
  // car model
  uint32_t shape = visualization_msgs::Marker::MESH_RESOURCE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;  //车身坐标系
  marker.header.stamp = ros::Time::now();
  marker.id = -1;
  marker.type = shape;
  // marker.mesh_resource = "package://data_conversion/models/car.dae";
  marker.mesh_resource = "file:///root/mviz_apa/src/utils/models/car.dae";
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.position.x = 1.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  tf::Quaternion q;
  q.setEuler(0, 0, 0);
  marker.pose.orientation.x = q[0];
  marker.pose.orientation.y = q[1];
  marker.pose.orientation.z = q[2];
  marker.pose.orientation.w = q[3];
  // modelPub.publish(marker);
  return marker;
}

visualization_msgs::Marker DrawTimeStamp3D(int64_t timestamp, std::string topic, std::string frame_id,
                                           std::string whichSeconds, float x, float y, float z) {
  // 时间戳显示，mm/yy hh:mm:ss
  auto timestamp_to_date_us = [](int64_t timestamp) -> std::string {
    auto mTime = std::chrono::microseconds(timestamp);
    auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm* now = std::gmtime(&tt);
    std::string year = std::to_string(now->tm_year + 1900) + "/";
    std::string month = std::to_string(now->tm_mon + 1) + "/";
    std::string day = std::to_string(now->tm_mday) + " ";
    std::string hour = std::to_string(now->tm_hour) + ":";
    std::string min = std::to_string(now->tm_min) + ":";
    std::string sec = std::to_string(now->tm_sec);
    return year.append(month).append(day).append(hour).append(min).append(sec);
  };
  auto timestamp_to_date_ms = [](int64_t timestamp) -> std::string {
    auto mTime = std::chrono::milliseconds(timestamp);
    auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm* now = std::gmtime(&tt);
    std::string year = std::to_string(now->tm_year + 1900) + "/";
    std::string month = std::to_string(now->tm_mon + 1) + "/";
    std::string day = std::to_string(now->tm_mday) + " ";
    std::string hour = std::to_string(now->tm_hour) + ":";
    std::string min = std::to_string(now->tm_min) + ":";
    std::string sec = std::to_string(now->tm_sec);
    return year.append(month).append(day).append(hour).append(min).append(sec);
  };
  auto timestamp_to_date_s = [](int64_t timestamp) -> std::string {
    auto mTime = std::chrono::seconds(timestamp);
    auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::seconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm* now = std::gmtime(&tt);
    std::string year = std::to_string(now->tm_year + 1900) + "/";
    std::string month = std::to_string(now->tm_mon + 1) + "/";
    std::string day = std::to_string(now->tm_mday) + " ";
    std::string hour = std::to_string(now->tm_hour) + ":";
    std::string min = std::to_string(now->tm_min) + ":";
    std::string sec = std::to_string(now->tm_sec);
    return year.append(month).append(day).append(hour).append(min).append(sec);
  };

  visualization_msgs::Marker textMarker;
  std::string timeStr;
  if ("us" == whichSeconds) {
    timeStr = timestamp_to_date_us(timestamp);
  } else if ("ms" == whichSeconds) {
    timeStr = timestamp_to_date_ms(timestamp);
  } else {
    timeStr = timestamp_to_date_s(timestamp);
  }
  textMarker.header.frame_id = frame_id;
  textMarker.header.stamp = ros::Time::now();
  textMarker.ns = topic;
  textMarker.id = -1;
  textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  textMarker.action = visualization_msgs::Marker::ADD;
  textMarker.lifetime = ros::Duration(0);
  textMarker.pose.position.x = x;
  textMarker.pose.position.y = y;
  textMarker.pose.position.z = z;
  textMarker.pose.orientation.w = 1.0;
  textMarker.scale.z = 2;
  textMarker.color.b = 0;
  textMarker.color.g = 0;
  textMarker.color.r = 1;
  textMarker.color.a = 1;
  textMarker.text = topic + ":" + timeStr;
  return textMarker;
}

int MakeDir(std::string dir) {
  int ret = 0;
  if (dir.empty()) {
    return -1;
  }
  if (dir[dir.size() - 1] != '/') {
    dir.push_back('/');
  }
  int len = dir.size();
  int j = 0;
  for (int i = 1; i < len; ++i) {
    if (dir[i] == '/') {
      std::string temp = dir.substr(0, i);
      if (access(temp.c_str(), F_OK) != 0) {
        // printf("MakeDir[%s]: %d: %s\n", dir.c_str(), j, temp.c_str());
        ret = mkdir(temp.c_str(), 0777);
        if (ret != 0) {
          return -1;
        }
        j++;
      }
    }
  }
  return 0;
}

int EnsureDir(std::string& dir, int limit_depth) {
  if (limit_depth < 0 && !dir.empty()) {
    return MakeDir(dir);
  }
  // 空字符串默认为当前路径即可
  if (dir.empty()) {
    dir = "./";
    return 0;
  }
  if (dir[dir.size() - 1] != '/') {
    dir.push_back('/');
  }

  int len = dir.size();
  int j = 0;
  int is_ok = true;
  for (int i = 1; i < len; ++i) {
    if (dir[i] == '/') {
      std::string temp = dir.substr(0, i);
      if (access(temp.c_str(), F_OK) != 0) {
        printf("CHECK TODO: MakeDir[%s]: %d: %s\n", dir.c_str(), j, temp.c_str());
        j++;
        if (j > limit_depth) {
          is_ok = false;
          printf("CHECK failed!!! MakeDir[%s], limit_depth:%d \n", dir.c_str(), limit_depth);
        }
      }
    }
  }

  if (is_ok) {
    return MakeDir(dir);
  } else {
    return -2;
  }
}

void SaveToJson(json minieye_obj, std::string dst_dir, std::string file_name) {
  EnsureDir(dst_dir, 5);
  std::string target_file = dst_dir + "/" + file_name;
  std::cout << target_file << std::endl;
  std::ofstream destFile(target_file, std::ios::out);
  destFile << std::setw(4) << minieye_obj << std::endl;
  destFile.close();
}
void SaveToJson(json minieye_obj, std::string file_name) {
  std::ofstream destFile(file_name, std::ios::out);
  destFile << std::setw(4) << minieye_obj << std::endl;
  destFile.close();
}

void PrintHead(const ImageFlowHeaderStruct& head) {
  std::cout << "--------head-------" << std::endl;
  std::cout << "head.Height:" << head.Height << std::endl;
  std::cout << "head.Width:" << head.Width << std::endl;
  std::cout << "head.SendTimeHigh:" << head.SendTimeHigh << std::endl;
  std::cout << "head.SendTimeLow:" << head.SendTimeLow << std::endl;
  std::cout << "head.FrameType:" << head.FrameType << std::endl;
  std::cout << "head.DataSize:" << head.DataSize << std::endl;
  std::cout << "head.Seq:" << head.Seq << std::endl;
  std::cout << "head.Sec:" << head.Sec << std::endl;
  std::cout << "head.Nsec:" << head.Nsec << std::endl;
  std::cout << "--------head-------" << std::endl;
  return;
}
