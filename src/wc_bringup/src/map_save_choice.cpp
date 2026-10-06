// Runs only after acquisition and control have stopped; never touches devices.
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFileInfo>
#include <QInputDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTextStream>

QJsonObject ask_map_save_choice(const QString & identity, const QString & destination) {
  QJsonObject result{{"decision", "pending"}, {"reason", "SAVE_DIALOG_CLOSED"}};
  QMessageBox box(QMessageBox::Question, QStringLiteral("是否保存本次建图结果？"),
    QStringLiteral("设备已停止。会话：%1\n保存：保留地图，实验模式同时保留原始数据。\n废弃：删除本次地图和本次原始记录，仅保留轻量诊断；历史实验不受影响。")
      .arg(identity), QMessageBox::NoButton);
  auto * save = box.addButton(QStringLiteral("保存"), QMessageBox::AcceptRole);
  auto * discard = box.addButton(QStringLiteral("废弃本次结果"), QMessageBox::DestructiveRole);
  auto * later = box.addButton(QStringLiteral("稍后决定"), QMessageBox::RejectRole);
  box.setDefaultButton(save); box.setEscapeButton(later);
  box.exec();
  if (box.clickedButton() == discard) {
    result = QJsonObject{{"decision", "discard"}, {"reason", "USER_DECLINED_SAVE"}};
  } else if (box.clickedButton() == save) {
    QString proposed = destination;
    while (true) {
      bool ok = false;
      const QString value = QInputDialog::getText(nullptr, QStringLiteral("保存位置"),
        QStringLiteral("请输入新的地图目录（不覆盖已有目录）："), QLineEdit::Normal, proposed, &ok).trimmed();
      if (!ok) {break;}
      const QFileInfo destination(value);
      if (value.isEmpty() || !destination.isAbsolute() || destination.exists() || destination.isSymLink()) {
        QMessageBox::warning(nullptr, QStringLiteral("保存位置无效"), QStringLiteral("请填写尚不存在的绝对目录路径。"));
        proposed = value; continue;
      }
      result = QJsonObject{{"decision", "save"}, {"destination", QDir::cleanPath(value)}};
      break;
    }
  }
  return result;
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
