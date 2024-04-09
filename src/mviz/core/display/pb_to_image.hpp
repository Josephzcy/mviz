// Copyright 2023 minieye
#ifndef PB_TO_IMAGE_HPP_
#define PB_TO_IMAGE_HPP_
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "RawInsParkingSpace.pb.h"
#include "parkingspace.pb.h"

namespace mviz1::data_visualization {

void DrawRawInsParkingSpace(cv::Mat &img, perception::RawInsParkingSpace &rps) {
  for (int i = 0; i < rps.ins_ps_size(); ++i) {
    auto &ps = rps.ins_ps(i);
    double confidence = ps.confidence();
    bool used = ps.occupied();
    // draw on image
    cv::Scalar cen_color(120, 120, 0);
    if (used) {
      cen_color = cv::Scalar(255, 0, 0);
    }
    // outside base point
    double cent_x = ps.center_pt().x();
    double cent_y = ps.center_pt().y();
    cv::putText(img, "0." + std::to_string(int(confidence * 100)),
                cv::Point(static_cast<int>(cent_x), static_cast<int>(cent_y)), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5,
                cen_color);

    // std::string info = "raw_ins_ps: entry_good-yellow entry_"
    //入位点  good黄色  非good红色 非入位点 good蓝色 非good紫色
    for (int j = 0; j < ps.corner_pts_size(); j++) {
      if (ps.corner_pts(j).is_entry()) {
        if (ps.corner_pts(j).is_good()) {
          cv::circle(img, cv::Point(static_cast<int>(ps.corner_pts(j).x()), static_cast<int>(ps.corner_pts(j).y())), 4,
                     YELLOW, -1);
        } else {
          cv::circle(img, cv::Point(static_cast<int>(ps.corner_pts(j).x()), static_cast<int>(ps.corner_pts(j).y())), 4,
                     RED, -1);
        }
      } else {
        if (ps.corner_pts(j).is_good()) {
          cv::circle(img, cv::Point(static_cast<int>(ps.corner_pts(j).x()), static_cast<int>(ps.corner_pts(j).y())), 4,
                     BLUE, -1);
        } else {
          cv::circle(img, cv::Point(static_cast<int>(ps.corner_pts(j).x()), static_cast<int>(ps.corner_pts(j).y())), 4,
                     PURPLE, -1);
        }
      }
    }
    for (int i = 0; i < ps.rod_pts_size(); ++i) {
      cv::circle(img, cv::Point(static_cast<int>(ps.rod_pts(i).x()), static_cast<int>(ps.rod_pts(i).y())), 4, ORANGE,
                 -1);
    }
  }
}
void DrawParkingSpace(cv::Mat &img, perception::ParkingSpace &ps) {
  static std::map<int, std::string> ParkslotTypeMap{{0, "Unknown"}, {1, "Vertical"}, {2, "Horizontal"}, {3, "Oblique"}};
  for (int i = 0; i < ps.parkslots_size(); i++) {
    auto color = ps.parkslots(i).occupied() ? RED : GREEN;
    std::vector<cv::Point> rect;
    for (auto &pt : ps.parkslots(i).corner_pts()) {
      rect.emplace_back(pt.image_pt().x(), pt.image_pt().y());
    }
    if (!rect.empty()) {
      cv::polylines(img, rect, true, color, 2);
      double fontScale = 0.5;  //字体缩放比
      int thickness = 0.5;
      std::string text = ParkslotTypeMap[ps.parkslots(i).type()];
      int baseline = 0;
      cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_TRIPLEX, fontScale, thickness, &baseline);
      baseline += thickness;

      cv::rectangle(
          img, cv::Rect(rect[0].x, rect[0].y - textSize.height - baseline, textSize.width, textSize.height + baseline),
          CYAN, 1);
      cv::putText(img, text, rect[0] - cv::Point(0, baseline), cv::FONT_HERSHEY_TRIPLEX, fontScale, BLACK, thickness,
                  8);
    }

    std::vector<cv::Point> rdpts;
    for (auto &pt : ps.parkslots(i).rod_pts()) {
      rdpts.emplace_back(pt.image_pt().x(), pt.image_pt().y());
    }
    if (rdpts.size() == 2) {
      cv::line(img, rdpts[0], rdpts[1], CHOCOLATE, 2);
    }
  }
}
void DrawFreeSpace(cv::Mat &fishImg, const freespacepoints::Points &fsPoints) {
  std::vector<cv::Point> fsPts;
  for (int i = 0; i < fsPoints.points_size(); i++) {
    fsPts.emplace_back(fsPoints.points(i).x(), fsPoints.points(i).y());
  }
  for (size_t i = 0; i < fsPts.size(); i++) {
    cv::circle(fishImg, fsPts[i], 4, CYAN, -1);
  }

  // cv::polylines(fishImg, fsPts, false, CYAN, 4);
}

}  // namespace mviz1::data_visualization
#endif  // PB_TO_IMAGE_HPP_