#include "log_view_ui.h"

SingleLogView::SingleLogView(QWidget *parent) : QWidget(parent) {
  QHBoxLayout *layout = new QHBoxLayout(this);
  treeView = new QTreeView(this);
  stmodel = new QStandardItemModel(this);
  treeView->setModel(stmodel);

  layout->addWidget(treeView);
  setLayout(layout);
  // ���ر�ͷ
  treeView->header()->setHidden(true);
}

void SingleLogView::UpdateLog(QString &total_text) {
  QStack<QStandardItem *> item_stack;
  QList<QStandardItem *> topList;

  for (QString text : total_text.split('\n')) {
    QStandardItem *item;
    if (text.endsWith("{")) {
      item = new QStandardItem(text.replace("{", ""));
      item_stack.push(item);
      continue;
    }

    if (text.endsWith("}")) {
      item = item_stack.pop();
    } else {
      item = new QStandardItem(text);
    }

    if (item_stack.isEmpty()) {
      topList.append(item);
    } else {
      item_stack.top()->appendRow(item);
    }
  }

  stmodel->clear();
  stmodel->appendColumn(topList);
}

LogView::LogView(QWidget *parent) : QWidget(parent) {
  QHBoxLayout *layout = new QHBoxLayout(this);
  for (size_t i = 0; i < tabNun; i++) {
    auto tabWidget = new QTabWidget(this);
    tabWidget->setTabPosition(QTabWidget::South);  // North, South, West, East
    layout->addWidget(tabWidget);
    tabWidgets.push_back(tabWidget);
  }
  // AddLogTabView("test1");
  // AddLogTabView("test2");
  // AddLogTabView("test3");
  // UpdateLogs("test1", "tick:123\ntime:465\n");
  // UpdateLogs("test2", "tick:123\ntime:465\n");
  // UpdateLogs("test3", "tick:123\ntime:465\n");
  setLayout(layout);
}

void LogView::UpdateLogs(std::string topic, QString logs) {
  auto it = log_views.find(topic);
  if (it != log_views.end()) {
    it->second->UpdateLog(logs);
  }
  return;
}

void LogView::AddLogTabView(std::string topic) {
  auto it = log_views.find(topic);
  if (it == log_views.end()) {
    SingleLogView *view = new SingleLogView(this);
    tabWidgets[tabIndex % tabNun]->addTab(view, QString(topic.c_str()));
    log_views[topic] = view;
    tabIndex++;
  }
  return;
}

void LogView::ClearLog() {
  QString kong = "";
  for (auto it = log_views.begin(); it != log_views.end(); ++it) {
    it->second->UpdateLog(kong);
  }
}
