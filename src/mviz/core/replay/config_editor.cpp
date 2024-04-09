#include "config_editor.h"

ConfigEditor::ConfigEditor(QWidget *parent) : QWidget(parent) {
  // 设置窗口属性为"关闭即销毁"
  setAttribute(Qt::WA_DeleteOnClose);
  setWindowTitle("Config Editor");
  // 创建按钮
  openButton = new QPushButton("Open", this);
  saveButton = new QPushButton("Save", this);
  saveAsButton = new QPushButton("Save As", this);

  // 创建文本编辑框
  textEdit = new QTextEdit(this);

  // 设置布局
  auto *hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(openButton);
  hlayout1->addWidget(saveButton);
  hlayout1->addWidget(saveAsButton);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addWidget(textEdit);
  setLayout(layout);

  // 连接按钮的点击事件到对应的槽函数
  connect(openButton, &QPushButton::clicked, this, &ConfigEditor::openConfigFile);
  connect(saveButton, &QPushButton::clicked, this, &ConfigEditor::saveConfigFile);
  connect(saveAsButton, &QPushButton::clicked, this, &ConfigEditor::saveAsConfigFile);
  connect(textEdit, &QTextEdit::textChanged, [&]() { saveButton->setText("save*"); });
}

void ConfigEditor::openConfigFile() {
  // 打开文件对话框，选择配置文件
  m_config_path = QFileDialog::getOpenFileName(this, "Open File", m_open_config_path, "YAML Files (*.yaml)");

  // 如果选择了文件，则读取文件内容并显示在文本编辑框中
  if (!m_config_path.isEmpty()) {
    QFileInfo fileInfo(m_config_path);
    m_open_config_path = fileInfo.absolutePath();
    QFile file(m_config_path);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream in(&file);
      textEdit->setText(in.readAll());
      file.close();
    } else {
      QMessageBox::warning(this, "Error", "Can not open");
    }
  }
}

void ConfigEditor::saveConfigFile() {
  // 如果选择了文件，则将文本编辑框中的内容保存到文件中
  if (!m_config_path.isEmpty()) {
    QFile file(m_config_path);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream out(&file);
      out << textEdit->toPlainText();
      file.close();
      QMessageBox::information(this, "Success", "File is Saved");
      saveButton->setText("Save");
    } else {
      QMessageBox::warning(this, "Error", "Can not save");
    }
  }
  // qDebug() << m_config_path;
}

void ConfigEditor::saveAsConfigFile() {
  // 打开文件对话框，选择保存文件的路径和名称
  m_config_save_path = QFileDialog::getSaveFileName(this, "Save as", m_open_config_path, "YAML Files (*.yaml)");

  // 如果选择了文件，则将文本编辑框中的内容保存到文件中
  if (!m_config_save_path.isEmpty()) {
    QFile file(m_config_save_path);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream out(&file);
      out << textEdit->toPlainText();
      file.close();
      QMessageBox::information(this, "Success", "File is Saved");
      saveButton->setText("Save");
    } else {
      QMessageBox::warning(this, "Error", "Can not save");
    }
  }
}

void ConfigEditor::setConfigPath(QString path) {
  if (path.isEmpty()) {
    return;
  }
  m_config_path = path;
  QFileInfo fileInfo(m_config_path);
  m_open_config_path = fileInfo.absolutePath();

  // 如果选择了文件，则读取文件内容并显示在文本编辑框中
  if (!m_config_path.isEmpty()) {
    QFile file(m_config_path);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream in(&file);
      textEdit->setText(in.readAll());
      file.close();
    } else {
      QMessageBox::warning(this, "Error", "Can not open");
    }
  }
}
