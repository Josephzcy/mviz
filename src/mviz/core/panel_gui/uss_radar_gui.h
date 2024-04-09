/***
 * @Copyright: Copyright 2023 MINIEYE
 * @Author: zhangchengyu zhangchengyu@minieye.cc
 * @Date: 2023-06-19 15:52:30
 * @LastEditors: zhangchengyu zhangchengyu@minieye.cc
 * @LastEditTime: 2023-06-19 16:08:48
 * @fileName: Do not edit
 * @FilePath: /mviz_apa/src/mviz_apa_show/src/uss_radar_gui.h
 * @Description:
 */
// Copy 2023 Minieye
#ifndef USS_RADAR_GUI_H_
#define USS_RADAR_GUI_H_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif
#include <QLabel>

#include "mviz_apa_show/UssRadar.h"
namespace mviz_apa {

struct UssRadarDisplay {
  float short_radar_fl;
  float short_radar_flm;
  float short_radar_frm;
  float short_radar_fr;

  float short_radar_bl;
  float short_radar_blm;
  float short_radar_brm;
  float short_radar_br;

  float long_radar_fl;
  float long_radar_bl;

  float long_radar_fr;
  float long_radar_br;
};

class UssRadarGui : public rviz::Panel {
  Q_OBJECT
 public:
  explicit UssRadarGui(QWidget* parent = nullptr);
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 Q_SIGNALS:
  void getOneMsg();
 public Q_SLOTS:

 protected Q_SLOTS:
  void UpdateUssRadarPanel();

 protected:
  void UssRadarCallBack(const mviz_apa_show::UssRadar& uss_radar_rosmsg);

  QLabel* short_radar_fl_;
  QLabel* short_radar_flm_;
  QLabel* short_radar_frm_;
  QLabel* short_radar_fr_;

  QLabel* short_radar_bl_;
  QLabel* short_radar_blm_;
  QLabel* short_radar_brm_;
  QLabel* short_radar_br_;

  QLabel* long_radar_fl_;
  QLabel* long_radar_bl_;

  QLabel* long_radar_fr_;
  QLabel* long_radar_br_;

  QLabel* short_radar_fl_value_;
  QLabel* short_radar_flm_value_;
  QLabel* short_radar_frm_value_;
  QLabel* short_radar_fr_value_;

  QLabel* short_radar_bl_value_;
  QLabel* short_radar_blm_value_;
  QLabel* short_radar_brm_value_;
  QLabel* short_radar_br_value_;

  QLabel* long_radar_fl_value_;
  QLabel* long_radar_bl_value_;

  QLabel* long_radar_fr_value_;
  QLabel* long_radar_br_value_;

 private:
  void UssRadarGuiLayout();
  UssRadarDisplay uss_radar_display_value_;
  ros::Subscriber sub_uss_radar_;
  ros::NodeHandle nh_;
};
}  // namespace mviz_apa

#endif  // USS_RADAR_GUI_H_