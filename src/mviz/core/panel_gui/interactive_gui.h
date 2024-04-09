#ifndef INTERACTION_GUI_H_
#define INTERACTION_GUI_H_

#ifndef Q_MOC_RUN
#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#endif
class QLineEdit;
namespace mviz_apa {
class CarBodyStatusGui : public rviz::Panel {
  Q_OBJECT
 public:
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
  CarBodyStatusGui(QWidget* parent = 0);

 public Q_SLOTS:
  void setTopic(const QString& topic);
 protected Q_SLOTS:
  void sendVel();
  void update_Linear_Velocity();
  void update_Angular_Velocity();
  void updateTopic();

 protected:
  QLineEdit* output_topic_editor_;
  QString output_topic_;

  QLineEdit* output_topic_editor_1;
  QString output_topic_1;

  QLineEdit* output_topic_editor_2;
  QString output_topic_2;

  ros::Publisher velocity_publisher_;
  ros::NodeHandle nh_;

  float linear_velocity_;
  float angular_velocity_;
};
}  // end namespace mviz_apa
#endif