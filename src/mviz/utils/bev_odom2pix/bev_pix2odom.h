#ifndef HAVP_PARKINGSPACE_POSTPROCESS_SERVICE_COMMON_BEV_PIX2WORLD_H_
#define HAVP_PARKINGSPACE_POSTPROCESS_SERVICE_COMMON_BEV_PIX2WORLD_H_

#include <opencv2/opencv.hpp>

#include "point.h"
#include "pose/pose.h"

namespace module {
namespace sensor_fusion {
namespace common {
class BevPix2Odom {
 public:
  BevPix2Odom(int img_width = 1920, int img_height = 1080, float half_width = 6, std::string car_info_path = "");
  ~BevPix2Odom();

  void Pix2Vehicle(Point2D& img_pt, Point3D& veh_pt);
  void Vehicle2Odom(Point3D& veh_pt, Point3D& odom_pt, Pose3D& pose);
  void Pix2Odom(Point2D& img_pt, Point3D& odom_pt, Pose3D& pose);

  void Odom2Vehicle(Point3D& odom_pt, Point3D& veh_pt, Pose3D& pose);
  void Vehicle2Pix(Point3D& veh_pt, Point2D& img_pt);
  void Odom2Pix(Point3D& odom_pt, Point2D& img_pt, Pose3D& pose);
  cv::Point Odom2Pix(Point3D& odom_pt, Pose3D& pose);

  int img_width_;
  int img_height_;
  float pix_size_;

 private:
  float half_width_;
  float half_height_;
  float center_diff_;
  Eigen::Matrix<double, 2, 2> R_;
  Eigen::Matrix<double, 2, 1> t_;
};
}  // namespace common
}  // namespace sensor_fusion
}  // namespace module
#endif  // HAVP_PARKINGSPACE_POSTPROCESS_SERVICE_COMMON_BEV_PIX2WORLD_H_