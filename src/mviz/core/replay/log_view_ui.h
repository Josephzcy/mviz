#ifndef _MVIZ_LOG_VIEW_UI_H_
#define _MVIZ_LOG_VIEW_UI_H_

#include <QDebug>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QList>
#include <QMessageBox>
#include <QPushButton>
#include <QStack>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QString>
#include <QTextEdit>
#include <QTextStream>
#include <QVBoxLayout>
#include <QWidget>
#include <QtWidgets>

#include "l2_types.h"

class SingleLogView : public QWidget {
  Q_OBJECT

 public:
  explicit SingleLogView(QWidget *parent = nullptr);
  void UpdateLog(QString &total_text);

 private:
  QTreeView *treeView = nullptr;
  QStandardItemModel *stmodel = nullptr;
};

class LogView : public QWidget {
  Q_OBJECT

 public:
  explicit LogView(QWidget *parent = nullptr);
  void UpdateLogs(std::string topic, QString logs);
  void AddLogTabViews(std::vector<std::string> &topics);
  void AddLogTabView(std::string);
  void ClearLog();

 private:
  std::vector<QTabWidget *> tabWidgets;
  std::map<std::string, SingleLogView *> log_views;
  int tabIndex = 0;
  size_t tabNun = 3;
};

#endif