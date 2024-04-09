#include "bev_pix2odom.h"

#include <glog/logging.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "json.hpp"

namespace module {
namespace sensor_fusion {
namespace common {

BevPix2Odom::BevPix2Odom(int img_width, int img_height, float half_width, std::string car_info_path) {
  img_width_ = img_width;
  img_height_ = img_height;
  half_width_ = half_width;
  half_width_ = 6;
  pix_size_ = half_width_ * 2 / img_width_;
  half_height_ = half_width_ * img_height_ / img_width_;

  nlohmann::json car_info_json;
  std::cout << "car_info_path : " << car_info_path << std::endl;
  std::ifstream json_file(car_info_path);
  if (!json_file.is_open()) {
    std::cerr << "[ERROR] car_info_path is not exist\n";
  }

  json_file >> car_info_json;

  float car_front_overhang = car_info_json["car_front_overhang"];
  float car_wheelbase = car_info_json["car_wheelbase"];
  float car_length = car_info_json["car_length"];

  center_diff_ = car_front_overhang + car_wheelbase - (car_length * 0.5);

  R_(0, 0) = 0;
  R_(0, 1) = -pix_size_;
  R_(1, 0) = -pix_size_;
  R_(1, 1) = 0;
  t_(0, 0) = half_height_;
  t_(1, 0) = half_width_;

  json_file.close();
}

BevPix2Odom::~BevPix2Odom() {}

void BevPix2Odom::Pix2Vehicle(Point2D& img_pt, Point3D& veh_pt) {
  Point2D res = R_ * img_pt + t_;
  veh_pt(0, 0) = res(0, 0) + center_diff_;
  veh_pt(1, 0) = res(1, 0);
  veh_pt(2, 0) = 0.0;
}

void BevPix2Odom::Pix2Odom(Point2D& img_pt, Point3D& odom_pt, Pose3D& pose) {
  Point3D veh_pt;
  Pix2Vehicle(img_pt, veh_pt);
  Vehicle2Odom(veh_pt, odom_pt, pose);
}

void BevPix2Odom::Vehicle2Odom(Point3D& veh_pt, Point3D& odom_pt, Pose3D& pose) {
  odom_pt = pose.rotation().matrix() * veh_pt + pose.translation();
}

void BevPix2Odom::Odom2Vehicle(Point3D& odom_pt, Point3D& veh_pt, Pose3D& pose) {
  veh_pt = pose.rotation().matrix().transpose() * (odom_pt - pose.translation());
}

void BevPix2Odom::Vehicle2Pix(Point3D& veh_pt, Point2D& img_pt) {
  Point2D res;
  res(0, 0) = veh_pt(0, 0) - center_diff_;
  res(1, 0) = veh_pt(1, 0);
  img_pt = R_.inverse() * (res - t_);
}

void BevPix2Odom::Odom2Pix(Point3D& odom_pt, Point2D& img_pt, Pose3D& pose) {
  Point3D veh_pt;
  Odom2Vehicle(odom_pt, veh_pt, pose);
  Vehicle2Pix(veh_pt, img_pt);
}

cv::Point BevPix2Odom::Odom2Pix(Point3D& odom_pt, Pose3D& pose) {
  Point3D veh_pt;
  Odom2Vehicle(odom_pt, veh_pt, pose);
  Point2D img_pt;
  Vehicle2Pix(veh_pt, img_pt);
  return cv::Point(img_pt(0, 0), img_pt(1, 0));  //@Notice Point2D是列向量
}

}  // namespace common
}  // namespace sensor_fusion
}  // namespace module