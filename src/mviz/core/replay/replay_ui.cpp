#include "replay_ui.h"

#include <ros/ros.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <cstdio>

#include "glog/logging.h"
#include "json.hpp"
#include "l2data_reader.h"
#include "msgpack.hpp"
// pb
#ifdef MVIZ_TDA4
#include "RawInsParkingSpace.pb.h"
#include "apa_gnss.pb.h"
#include "apa_state.pb.h"
#include "avm_settings.pb.h"
#include "camera.pb.h"
#include "freespace_obstacles.pb.h"
#include "freespacepoints.pb.h"
#include "hmi_to_soc.pb.h"
#include "ihu_to_soc.pb.h"
#include "imu.pb.h"
#include "odo_vehicle_signal.pb.h"
#include "odometry_3d.pb.h"
#include "parking_gnss.pb.h"
#include "parking_ins.pb.h"
#include "parkingspace.pb.h"
#include "planning.pb.h"
#include "planningtohmi.pb.h"
#include "soc_to_ihu.pb.h"
#include "ultra_radar.pb.h"
#include "vehicle_control.pb.h"
#include "vehicle_signal.pb.h"
#include "vtr.pb.h"
#endif

#ifdef MVIZ_J3
#include "RawInsParkingSpace.pb.h"
#include "apa_odometry.pb.h"
#include "apa_state.pb.h"
#include "avm_calib_ctrl.pb.h"
#include "avm_calib_param.pb.h"
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
#include "vtr.pb.h"
#endif

using namespace mviz::replay;

namespace mviz_replay {

ReplayUI::ReplayUI(QWidget* parent) : QWidget(parent) {
  SetUI();
  SetReplayConfig();

  QObject::connect(m_open_config_button, &QPushButton::clicked, this, &ReplayUI::on_open_config_button_clicked);

  QObject::connect(m_play_start_stop_button, &QPushButton::clicked, this, &ReplayUI::on_play_start_stop_button_clicked);
  connect(m_play_back_button, &QPushButton::clicked, this, &ReplayUI::on_play_back_button_clicked);
  connect(m_play_next_button, &QPushButton::clicked, this, &ReplayUI::on_play_next_button_clicked);
  connect(m_play_restart_button, &QPushButton::clicked, this, &ReplayUI::on_play_restart_button_clicked);
  connect(m_play_slider, &QSlider::sliderPressed, this, &ReplayUI::on_play_slider_pressed);
  connect(m_play_slider, &QSlider::sliderReleased, this, &ReplayUI::on_play_slider_released);
  connect(m_play_slider, &QSlider::valueChanged, this, &ReplayUI::on_play_slider_value_changed);

  m_pTimer = new QTimer;
  m_pTimer->setInterval(50);
  connect(m_pTimer, SIGNAL(timeout()), this, SLOT(playTimer()));

  connect(m_config_button, &QPushButton::clicked, [&]() {
    ConfigEditor* config_editor = new ConfigEditor();
    config_editor->setConfigPath(m_config_path);
    config_editor->show();
  });
  // connect(this, &ReplayUI::replay_config_changed, this, &ReplayUI::on_replay_config_changed);
}

void ReplayUI::SetUI() {
  m_config_button = new QPushButton(this);
  m_config_button->setText("EditConfig");

  // m_open_data_dir_button = new QPushButton(this);
  // m_open_data_dir_button->setText("OpenData");

  m_open_config_button = new QPushButton(this);
  m_open_config_button->setText("SelectConfig");

  // m_file_dir_lineedit = new QLineEdit(this);
  m_file_dir_label = new QLabel(this);
  m_file_dir_label->setText("KeyTopic: DataPath:");

  m_config_path_label = new QLabel(this);
  m_config_path_label->setText("ConfigPath:" + m_config_path);

  m_play_start_stop_button = new QPushButton(this);
  m_play_start_stop_button->setText("StartPlay");

  m_play_back_button = new QPushButton(this);
  m_play_back_button->setText("<<");
  m_play_back_button->setEnabled(false);

  m_play_next_button = new QPushButton(this);
  m_play_next_button->setText(">>");
  m_play_next_button->setEnabled(false);

  m_play_restart_button = new QPushButton(this);
  m_play_restart_button->setText("Restart");

  // m_play_slider = new QSlider(Qt::Horizontal, this);
  m_play_slider = new ClickableSlider(Qt::Horizontal, this);

  m_current_idx_label = new QLabel(this);
  m_current_idx_label->setText("Current idx: 0");

  m_fps_label = new QLabel("FPS:0", this);

  m_log_view = new LogView(this);

  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(m_config_button);
  hlayout1->addWidget(m_open_config_button);
  hlayout1->addWidget(m_play_start_stop_button);
  hlayout1->addWidget(m_play_back_button);
  hlayout1->addWidget(m_play_next_button);
  hlayout1->addWidget(m_play_restart_button);

  // hlayout1->addWidget(m_open_data_dir_button);

  auto* hlayout2 = new QHBoxLayout;
  hlayout1->addWidget(m_play_slider);
  hlayout1->addWidget(m_current_idx_label);
  hlayout1->addWidget(m_fps_label);

  QFrame* line = new QFrame();
  line->setFrameShape(QFrame::VLine);
  line->setFrameShadow(QFrame::Sunken);

  auto* hlayout3 = new QHBoxLayout;
  hlayout3->addWidget(m_config_path_label);
  hlayout3->addWidget(line);
  hlayout3->addWidget(m_file_dir_label);

  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  // layout->addLayout(hlayout2);
  layout->addLayout(hlayout3);
  layout->addWidget(m_log_view);
  setLayout(layout);
}

void ReplayUI::on_open_config_button_clicked() {
  if (is_play)  //正在播放则暂停
  {
    on_play_start_stop_button_clicked();
  }
  auto selected_path = QFileDialog::getOpenFileName(this, "Selete Config", m_open_config_path,
                                                    //   "All Files (*.*);;Text Files (*.yaml)");
                                                    "YAML Files (*.yaml)");
  if (!selected_path.isEmpty()) {
    m_config_path_label->setText("ConfigPath:" + selected_path);
    m_config_path = selected_path;
    QFileInfo fileInfo(m_config_path);
    m_open_config_path = fileInfo.absolutePath();
    SetReplayConfig();
    process_index = 0;
  }
  on_play_start_stop_button_clicked();  //还完config 直接播
}

void ReplayUI::on_open_data_button_clicked() {
  if (is_play) {
    on_play_start_stop_button_clicked();
  }
  auto selected_path = QFileDialog::getExistingDirectory(this, "Selete Dataset Dir", m_open_data_dir,
                                                         QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if (!selected_path.isEmpty()) {
    m_file_dir_label->setText(selected_path);
    m_data_dir = selected_path;
    m_open_data_dir = selected_path;
  }
}

void ReplayUI::on_play_start_stop_button_clicked() {
  if (IsReplayConfigYmlExists()) {
    is_play = !is_play;
    if (!is_play) {
      m_pTimer->stop();
      m_play_start_stop_button->setText("Play");
      m_play_back_button->setEnabled(true);
      m_play_next_button->setEnabled(true);
      // m_open_data_dir_button->setEnabled(true);
    } else {
      m_pTimer->start();
      m_play_start_stop_button->setText("Pause");
      m_play_back_button->setEnabled(false);
      m_play_next_button->setEnabled(false);
      // m_open_data_dir_button->setEnabled(false);
    }
  }
  if (!IsReplayConfigYmlExists()) {
    qWarning() << "[ERROR] config not exist:" << m_config_path;
  }
}

void ReplayUI::on_play_back_button_clicked() {
  if (process_index <= 0) {
    qDebug() << "[Warn] This is the first frame!";
    return;
  }

  if (!is_play) {
    process_index--;
    playTimer();
    process_index--;
  }
}
void ReplayUI::on_play_next_button_clicked() {
  if (process_index >= dataset_length) {
    qDebug() << "[Warn] This is the last frame!";
    return;
  }

  if (!is_play) {
    playTimer();
  }
}

void ReplayUI::on_play_restart_button_clicked() {
  process_index = 0;
  if (!is_play) {
    on_play_start_stop_button_clicked();
  }
}

void ReplayUI::on_play_slider_pressed() {
  // if(is_play && m_play_start_stop_button->isEnabled())
  // {
  //   on_play_start_stop_button_clicked();
  // }
  if (is_play) {
    m_pTimer->stop();
  }
}

void ReplayUI::on_play_slider_released() {
  playTimer();
  if (is_play) {
    m_pTimer->start();
  }
}

void ReplayUI::on_play_slider_value_changed(int value) {
  process_index = value;
  m_current_idx_label->setText(current_idx_str + QString::number(process_index));
}

void ReplayUI::playTimer() {
  if (m_data_readmanager == nullptr) {
    qDebug() << "[Error] ReaderManager is nullptr!";
    on_play_start_stop_button_clicked();
    return;
  }

  if (m_data_pubmanager == nullptr) {
    qDebug() << "[Error] PubManager is nullptr!";
    on_play_start_stop_button_clicked();
    return;
  }

  if (!m_data_readmanager->is_available) {
    qDebug() << "[Error] ReaderManager is not available, check config!";
    on_play_start_stop_button_clicked();
    return;
  }

  auto frames = m_data_readmanager->GetFrames(process_index);
  m_data_pubmanager->PublishFrame(frames, process_index);
  UpdateLogView(frames);

  m_play_slider->setValue(process_index);
  m_current_idx_label->setText(current_idx_str + QString::number(process_index));
  if (process_index < dataset_length) {
    process_index++;
  } else {
    on_play_start_stop_button_clicked();
  }

  fps_count_++;
  auto fps_now = std::chrono::steady_clock::now();
  // qDebug() << "playTimer cost:" << std::chrono::duration_cast<std::chrono::duration<double>>(fps_now - now).count();
  std::chrono::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(fps_now - fps_start_);
  if (time_used.count() > 1 && fps_count_ > 0) {
    fps_ = fps_count_ / time_used.count();
    fps_count_ = 0;
    fps_start_ = fps_now;
    // qDebug() <<"rma fps " << fps_;
    QString fps_info = QString("FPS:") + QString::number(int(fps_));
    m_fps_label->setText(fps_info);
  }
}

void ReplayUI::on_replay_config_changed() { SetReplayConfig(); }

void ReplayUI::SetReplayConfig() {
  if (IsReplayConfigYmlExists()) {
    mviz::replay::ConfigurationManager config_manager(m_config_path.toStdString());
    g_mrcfg.reset(new mviz::replay::MvizReplayConfig);
    *g_mrcfg = config_manager.GetConfig();

    m_data_dir = QString(g_mrcfg->reader_manager_config.mviz_data_path.c_str());
    auto data_info = "KeyTopic:" + g_mrcfg->reader_manager_config.key_topic +
                     " | DataPath:" + g_mrcfg->reader_manager_config.mviz_data_path;
    m_file_dir_label->setText(data_info.c_str());

    // 检查 数据路径是否存在
    if (!IsReplayDataDirExists()) {
      qDebug() << "[Error] Mviz Data path not Exists!";
      if (m_data_readmanager != nullptr) m_data_readmanager->is_available = false;  //让 playTimer 不执行
      return;
    }

    //当参数变了，这两个类需要重置
    if (m_data_readmanager != nullptr) {
      m_data_readmanager.reset(
          new mviz::replay::ReaderManager);  // @TODO FIX这里会卡死界面，暂时无解，回放另一个数据时需要重启
    } else {
      m_data_readmanager = std::make_shared<mviz::replay::ReaderManager>();
    }
    if (m_data_pubmanager != nullptr) {
      m_data_pubmanager.reset(new mviz::replay::FramesPubManager);
    } else {
      m_data_pubmanager = std::make_shared<mviz::replay::FramesPubManager>();
    }
    // qDebug() <<"Func:"<< __FUNCTION__ << ",line:" << __LINE__;
    dataset_length = m_data_readmanager->GetKeyTopicDataLen() - 5;
    m_play_slider->setRange(0, dataset_length - 1);  // 设置滑块的范围
    m_play_slider->setValue(0);                      // 设置初始值
    for (auto& topic_config : g_mrcfg->reader_manager_config.topic_configs) {
      if (IsImgTopic(topic_config.topic)) {
        continue;
      }
      m_log_view->AddLogTabView(topic_config.topic);
    }
    m_log_view->ClearLog();
    qDebug() << "[INFO] set replay config finish";
  }
}

bool ReplayUI::IsReplayConfigYmlExists() {
  if (m_config_path.isEmpty()) {
    return false;
  }
  QFile file(m_config_path);
  return file.exists();
}

bool ReplayUI::IsReplayDataDirExists() {
  if (m_data_dir.isEmpty()) {
    return false;
  }
  QDir directory(m_data_dir);
  return directory.exists();
}

template <typename MessageType>
void UpdateLogViewForTopic(const std::string& topicName, const L2DataFramePtr& frame, LogView* logView) {
  MessageType pb;
  pb.ParseFromArray(frame->data.data(), frame->data.size());
  logView->UpdateLogs(topicName, pb.Utf8DebugString().c_str());
}

void ReplayUI::UpdateLogView(L2DataFramePtrList frames) {
  for (const auto& frame : frames) {
    const std::string& topicName = frame->topic;

#ifdef MVIZ_TDA4
    if (topicName == "vehicle_signal") {
      UpdateLogViewForTopic<minieye::VehicleSignal>(topicName, frame, m_log_view);
    } else if (topicName == "odo_vehicle_signal") {
      UpdateLogViewForTopic<minieye::OdoVehicleSignal>(topicName, frame, m_log_view);
    } else if (topicName == "apa_vehicle_signal") {
      UpdateLogViewForTopic<minieye::ApaVehicleSignal>(topicName, frame, m_log_view);
    } else if (topicName == "imu") {
      UpdateLogViewForTopic<minieye::ImuDataList>(topicName, frame, m_log_view);
    } else if (topicName == "apa_gnss") {
      UpdateLogViewForTopic<minieye::GnssData>(topicName, frame, m_log_view);
    } else if (topicName == "uss") {
      UpdateLogViewForTopic<minieye::UltrasonicRadar>(topicName, frame, m_log_view);
    } else if (topicName == "apa_state_control") {
      UpdateLogViewForTopic<minieye::APAStateControl>(topicName, frame, m_log_view);
    } else if (topicName == "odometry_3d" || topicName == "odometry_3d_gt" || topicName == "odometry_3d_lz") {
      UpdateLogViewForTopic<minieye::Odometry3D>(topicName, frame, m_log_view);
    } else if (topicName == "raw_ins_ps") {
      UpdateLogViewForTopic<perception::RawInsParkingSpace>(topicName, frame, m_log_view);
    } else if (topicName == "freespace") {
      UpdateLogViewForTopic<freespacepoints::FreespacePoints>(topicName, frame, m_log_view);
    } else if (topicName == "gridmap") {
      UpdateLogViewForTopic<perception::FreespaceObstacles>(topicName, frame, m_log_view);
    } else if (topicName == "parkingspace" || topicName == "parkingspace_mviz" || topicName == "parkingspace_to_ui" ||
               topicName == "vis_parkingspace" || topicName == "vis_parkingspace_to_ui" ||
               topicName == "cus_parkingspace" || topicName == "uss_parkingspace_mviz") {
      UpdateLogViewForTopic<perception::ParkingSpace>(topicName, frame, m_log_view);
    } else if (topicName == "planning") {
      UpdateLogViewForTopic<minieye::Planning>(topicName, frame, m_log_view);
    } else if (topicName == "planning_to_hmi") {
      UpdateLogViewForTopic<minieye::PlanningToHMI>(topicName, frame, m_log_view);
    } else if (topicName == "vehicle_control") {
      UpdateLogViewForTopic<minieye::VehicleControl>(topicName, frame, m_log_view);
    } else if (topicName == "vtr") {
      UpdateLogViewForTopic<minieye::Vtr>(topicName, frame, m_log_view);
    } else if (topicName == "hmi_to_soc_havp") {
      UpdateLogViewForTopic<minieye::HmiToSoc>(topicName, frame, m_log_view);
    } else if (topicName == "ihu_to_soc") {
      UpdateLogViewForTopic<minieye::IHUToSoc>(topicName, frame, m_log_view);
    } else if (topicName == "avm_settings") {
      UpdateLogViewForTopic<minieye::AVMSettings>(topicName, frame, m_log_view);
    } else if (topicName == "soc_to_ihu") {
      UpdateLogViewForTopic<minieye::SocToIHU>(topicName, frame, m_log_view);
    } else if (topicName == "parking_ins") {
      UpdateLogViewForTopic<minieye::parking::AsensingINSData>(topicName, frame, m_log_view);
    } else if (topicName == "parking_gnss") {
      UpdateLogViewForTopic<minieye::parking::AsensingGNSSData>(topicName, frame, m_log_view);
    }
#endif

#ifdef MVIZ_J3
    if (topicName == "raw_ins_ps") {
      UpdateLogViewForTopic<perception::RawInsParkingSpace>(topicName, frame, m_log_view);
    } else if (topicName == "odometry") {
      UpdateLogViewForTopic<apa::ApaOdometry>(topicName, frame, m_log_view);
    } else if (topicName == "apa_state_control") {
      UpdateLogViewForTopic<minieye::APAStateControl>(topicName, frame, m_log_view);
    } else if (topicName == "avm_calib_ctrl") {
      UpdateLogViewForTopic<minieye::AVMCalibCtrl>(topicName, frame, m_log_view);
    } else if (topicName == "gridmap") {
      UpdateLogViewForTopic<perception::FreespaceObstacles>(topicName, frame, m_log_view);
    } else if (topicName == "freespace") {
      UpdateLogViewForTopic<freespacepoints::FreespacePoints>(topicName, frame, m_log_view);
    } else if (topicName == "odometry_3d" || topicName == "odometry_3d_gt" || topicName == "odometry_3d_lz") {
      UpdateLogViewForTopic<minieye::Odometry3D>(topicName, frame, m_log_view);
    } else if (topicName == "parkingspace" || topicName == "parkingspace_mviz" || topicName == "parkingspace_to_ui" ||
               topicName == "vis_parkingspace" || topicName == "vis_parkingspace_to_ui" ||
               topicName == "cus_parkingspace" || topicName == "uss_parkingspace_mviz") {
      UpdateLogViewForTopic<perception::ParkingSpace>(topicName, frame, m_log_view);
    } else if (topicName == "planning") {
      UpdateLogViewForTopic<minieye::Planning>(topicName, frame, m_log_view);
    } else if (topicName == "planning_to_hmi") {
      UpdateLogViewForTopic<minieye::PlanningToHMI>(topicName, frame, m_log_view);
    } else if (topicName == "uss") {
      UpdateLogViewForTopic<minieye::UltrasonicRadar>(topicName, frame, m_log_view);
    } else if (topicName == "vehicle_control") {
      UpdateLogViewForTopic<minieye::VehicleControl>(topicName, frame, m_log_view);
    } else if (topicName == "vehicle_signal") {
      UpdateLogViewForTopic<minieye::VehicleSignal>(topicName, frame, m_log_view);
    } else if (topicName == "vtr") {
      UpdateLogViewForTopic<minieye::Vtr>(topicName, frame, m_log_view);
    }
#endif
  }
}

}  // end of namespace mviz_replay
