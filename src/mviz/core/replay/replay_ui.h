#ifndef Mviz_REPLAY_UI_H_
#define Mviz_REPLAY_UI_H_

#include <ros/ros.h>

#include <QComboBox>
#include <QDebug>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QtGlobal>

#include "clickable_slider.h"
#include "config_editor.h"
#include "configuration_manager.h"
#include "frames_pub_manager.h"
#include "global_value.h"
#include "log_view_ui.h"
#include "reader_manager.h"

using ReaderManagerPtr = std::shared_ptr<mviz::replay::ReaderManager>;
using FramesPubManagerPtr = std::shared_ptr<mviz::replay::FramesPubManager>;

namespace mviz_replay {

class ReplayUI : public QWidget {
  Q_OBJECT
 public:
  explicit ReplayUI(QWidget *parent = nullptr);
  void SetUI();

 Q_SIGNALS:
  // void getOneMsg();
  void replay_config_changed();
 public Q_SLOTS:
  void on_replay_config_changed();

 protected Q_SLOTS:
  void on_open_config_button_clicked();
  void on_open_data_button_clicked();  // 弃用
  void on_play_start_stop_button_clicked();
  void on_play_back_button_clicked();
  void on_play_next_button_clicked();
  void on_play_restart_button_clicked();

  void on_play_slider_pressed();
  void on_play_slider_released();
  void on_play_slider_value_changed(int);

  void playTimer();

 protected:
 private:
  QPushButton *m_config_button = nullptr;

  // QPushButton *m_open_data_dir_button = nullptr;
  QString m_data_dir{""};
  QString m_open_data_dir{"./"};
  QLabel *m_file_dir_label = nullptr;

  // QLineEdit *m_file_dir_lineedit = nullptr;
  QPushButton *m_open_config_button = nullptr;
  QLabel *m_config_path_label = nullptr;
  QString m_config_path{""};
  // QString m_open_config_path{"/home/nico/minieye/mviz/mviz2_ros/src/mviz/config"};
  QString m_open_config_path{"/root/mviz2_ros/src/mviz/config"};

  QPushButton *m_play_start_stop_button = nullptr;
  QPushButton *m_play_back_button = nullptr;
  QPushButton *m_play_next_button = nullptr;
  QPushButton *m_play_restart_button = nullptr;

  QLabel *m_current_idx_label = nullptr;
  QString current_idx_str{"Current idx: "};
  // QSlider *m_play_slider = nullptr;
  ClickableSlider *m_play_slider = nullptr;
  QTimer *m_pTimer = nullptr;

  float fps_{0};
  int fps_count_{0};
  QLabel *m_fps_label;
  std::chrono::steady_clock::time_point fps_start_ = std::chrono::steady_clock::now();

  bool is_play = false;
  int dataset_length = 0;
  int process_index = 0;

  bool config_exists{false};

  LogView *m_log_view;

 private:
  void SetReplayConfig();
  bool IsReplayConfigYmlExists();
  bool IsReplayDataDirExists();
  void UpdateLogView(mviz::replay::L2DataFramePtrList);

  ReaderManagerPtr m_data_readmanager = nullptr;
  FramesPubManagerPtr m_data_pubmanager = nullptr;
};

}  // end of namespace mviz_replay

#endif
