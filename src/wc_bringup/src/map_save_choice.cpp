// Runs only after acquisition and control have stopped; never touches devices.
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFileInfo>
#include <QFileDialog>
#include <QInputDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTextStream>
#include <QVBoxLayout>
#include <QShowEvent>

namespace {
// QInputDialog builds its internal layout lazily when it becomes visible.
// Before exec()/show(), even after setTextValue(), layout() is null in Qt 5.15.
// Keep the dialog's native validation/buttons; add browsing only after the
// base class has established its widgets. Repeated show events add it once.
class SavePathInput final : public QInputDialog {
public:
  explicit SavePathInput(const QString & identity) : identity_(identity) {
    setObjectName(QStringLiteral("map_save_path_dialog"));
    setOkButtonText(QStringLiteral("确认保存"));
    setCancelButtonText(QStringLiteral("返回上一步"));
  }
protected:
  void showEvent(QShowEvent * event) override {
    QInputDialog::showEvent(event);
    if (browse_) {return;}
    auto * initialized_layout = layout();
    if (!initialized_layout) {
      // Defensive failure is visible and leaves native manual path entry
      // usable; never dereference an uninitialized Qt private layout.
      qCritical("QInputDialog has no layout after show; folder browser unavailable");
      return;
    }
    browse_ = new QPushButton(QStringLiteral("浏览文件夹…"), this);
    browse_->setObjectName(QStringLiteral("map_save_browse_button"));
    initialized_layout->addWidget(browse_);
    // The parent's first child-show pass already ran before showEvent. A new
    // child created here otherwise stays hidden until a later layout event.
    browse_->show();
    initialized_layout->activate();
    adjustSize();
    connect(browse_, &QPushButton::clicked, this, [this]() {
      const auto parent = QFileDialog::getExistingDirectory(this, QStringLiteral("选择地图保存的父文件夹"),
        QFileInfo(textValue()).dir().absolutePath(), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
      if (!parent.isEmpty()) {setTextValue(QDir(parent).filePath("map_" + identity_));}
    });
  }
private:
  QString identity_;
  QPushButton * browse_{};
};
}  // namespace

QJsonObject ask_map_save_choice(const QString & identity, const QString & destination) {
  QJsonObject result{{"decision", "pending"}, {"reason", "SAVE_DIALOG_CLOSED"}};
  while (true) {
  QMessageBox box(QMessageBox::Question, QStringLiteral("是否保存本次建图结果？"),
    QStringLiteral("设备已停止。会话：%1\n保存：选择路径，保留本次地图及建图会话数据。\n不保存：丢弃本次临时地图及本次临时原始录包，无法再回放本次临时记录。独立数据录制和历史结果不受影响。")
      .arg(identity), QMessageBox::NoButton);
  auto * save = box.addButton(QStringLiteral("保存"), QMessageBox::AcceptRole);
  auto * discard = box.addButton(QStringLiteral("不保存"), QMessageBox::DestructiveRole);
  auto * later = box.addButton(QStringLiteral("稍后决定"), QMessageBox::RejectRole);
  box.setDefaultButton(save); box.setEscapeButton(later);
  box.exec();
  if (box.clickedButton() == discard) {
    result = QJsonObject{{"decision", "discard"}, {"reason", "USER_DECLINED_SAVE"}};
    return result;
  } else if (box.clickedButton() == save) {
    QString proposed = destination;
    while (true) {
      SavePathInput input(identity);
      input.setWindowTitle(QStringLiteral("保存位置"));
      input.setLabelText(QStringLiteral("选择父文件夹或填写新的地图目录（不覆盖已有目录）："));
      input.setInputMode(QInputDialog::TextInput);
      input.setTextValue(proposed);
      if (input.exec() != QDialog::Accepted) {break;}  // Return to save/discard, never discard on cancel.
      const QString value = input.textValue().trimmed();
      const QFileInfo destination(value);
      if (value.isEmpty() || !destination.isAbsolute() || destination.exists() || destination.isSymLink()) {
        QMessageBox::warning(nullptr, QStringLiteral("保存位置无效"), QStringLiteral("请填写尚不存在的绝对目录路径。"));
        proposed = value; continue;
      }
      result = QJsonObject{{"decision", "save"}, {"destination", QDir::cleanPath(value)}};
      return result;
    }
  } else {return result;}
  }
}

#ifndef WC_SAVE_CHOICE_TEST
int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  QCommandLineParser parser;
  parser.addHelpOption();
  parser.addOption({QStringLiteral("session-id"), QStringLiteral("Session identity"), QStringLiteral("id")});
  parser.addOption({QStringLiteral("default-destination"), QStringLiteral("New output directory"), QStringLiteral("path")});
  parser.process(app);
  const auto result=ask_map_save_choice(parser.value(QStringLiteral("session-id")), parser.value(QStringLiteral("default-destination")));
  QTextStream(stdout) << QJsonDocument(result).toJson(QJsonDocument::Compact) << '\n';
  return 0;
}
#endif
