#ifndef _MVIZ_CONFIG_EDITOR_H_
#define _MVIZ_CONFIG_EDITOR_H_
#include <QDebug>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QTextStream>
#include <QVBoxLayout>
#include <QWidget>

class ConfigEditor : public QWidget {
  Q_OBJECT
 public:
  ConfigEditor(QWidget *parent = nullptr);
  void setConfigPath(QString path);

 private Q_SLOTS:
  void openConfigFile();

  void saveConfigFile();

  void saveAsConfigFile();

 private:
  QTextEdit *textEdit;
  QPushButton *openButton;
  QPushButton *saveButton;
  QPushButton *saveAsButton;
  QString m_config_path;
  QString m_config_save_path;
  QString m_open_config_path;

 protected:
};

#endif