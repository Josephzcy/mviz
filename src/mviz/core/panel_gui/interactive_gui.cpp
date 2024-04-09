
#include "panel_gui/interactive_gui.h"

#include <geometry_msgs/Twist.h>
#include <stdio.h>

#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>
#include <QVBoxLayout>

namespace mviz_apa {

CarBodyStatusGui::CarBodyStatusGui(QWidget* parent) : rviz::Panel(parent), linear_velocity_(0), angular_velocity_(0) {
  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget(new QLabel("apa mode:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  topic_layout->addWidget(new QLabel("grar mode:"));
  output_topic_editor_1 = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_1);

  topic_layout->addWidget(new QLabel("yaw bias:"));
  output_topic_editor_2 = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_2);
  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);

  QTimer* output_timer = new QTimer(this);

  connect(output_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
  connect(output_topic_editor_1, SIGNAL(editingFinished()), this, SLOT(update_Linear_Velocity()));
  connect(output_topic_editor_2, SIGNAL(editingFinished()), this, SLOT(update_Angular_Velocity()));

  connect(output_timer, SIGNAL(timeout()), this, SLOT(sendVel()));
  output_timer->start(100);
}

void CarBodyStatusGui::update_Linear_Velocity() {
  QString temp_string = output_topic_editor_1->text();
  float lin = temp_string.toFloat();
  linear_velocity_ = lin;
}

void CarBodyStatusGui::update_Angular_Velocity() {
  QString temp_string = output_topic_editor_2->text();
  float ang = temp_string.toFloat();
  angular_velocity_ = ang;
}

void CarBodyStatusGui::updateTopic() { setTopic(output_topic_editor_->text()); }

void CarBodyStatusGui::setTopic(const QString& new_topic) {
  if (new_topic != output_topic_) {
    output_topic_ = new_topic;

    if (output_topic_ == "") {
      velocity_publisher_.shutdown();
    } else {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_topic_.toStdString(), 1);
    }
    Q_EMIT configChanged();
  }
}

void CarBodyStatusGui::sendVel() {
  if (ros::ok() && velocity_publisher_) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish(msg);
  }
}

void CarBodyStatusGui::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
}

void CarBodyStatusGui::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    output_topic_editor_->setText(topic);
    updateTopic();
  }
}
}  // namespace mviz_apa

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mviz_apa::CarBodyStatusGui, rviz::Panel)
