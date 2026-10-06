// SPDX-License-Identifier: Apache-2.0
#include "wc_bringup/mapping_health.hpp"
#include <QDateTime>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
namespace wc_bringup {
namespace {
QByteArray read_bounded(const QString & path) {
  QFile file(path);
  if (QFileInfo(path).isSymLink() || !file.open(QIODevice::ReadOnly) || file.size() > 1000000) {return {};}
  return file.read(1000000);
}
}
MappingHealthPanel::MappingHealthPanel(const QStringList & arguments, QWidget * parent)
: QWidget(parent) {
  setObjectName(QStringLiteral("mapping_health_panel"));
  QString view;
  for (int i = 1; i < arguments.size(); ++i) {
    if ((arguments[i] == QStringLiteral("-d") || arguments[i] == QStringLiteral("--display-config")) && i + 1 < arguments.size()) {
      view = arguments[++i];
    } else if (arguments[i].startsWith(QStringLiteral("--display-config="))) {
      view = arguments[i].mid(QStringLiteral("--display-config=").size());
    }
  }
  QFileInfo info(view);
  if (info.isAbsolute() && info.isFile() && !info.isSymLink()) {
    directory_ = info.absolutePath();
    const auto session = QJsonDocument::fromJson(read_bounded(QDir(directory_).filePath(QStringLiteral("session.json")))).object();
    identity_ = session.value(QStringLiteral("session_id")).toString();
  }
  auto * row = new QHBoxLayout(this); row->setContentsMargins(8, 3, 8, 3);
  summary_ = new QLabel(QStringLiteral("健康诊断：等待报告；地图质量未独立验收。"), this);
  summary_->setObjectName(QStringLiteral("mapping_health_summary"));
  summary_->setTextFormat(Qt::PlainText); summary_->setWordWrap(true);
  reasons_ = new QPushButton(QStringLiteral("查看原因"), this);
  reasons_->setObjectName(QStringLiteral("mapping_health_reasons"));
  row->addWidget(summary_, 1); row->addWidget(reasons_);
  connect(reasons_, &QPushButton::clicked, this, [this]() {
    auto * dialog = new QDialog(this); dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(QStringLiteral("健康诊断与复测依据")); dialog->resize(760, 540);
    auto * layout = new QVBoxLayout(dialog); auto * text = new QPlainTextEdit(dialog);
    text->setReadOnly(true); text->setPlainText(details_); layout->addWidget(text); dialog->show();
  });
  timer_ = new QTimer(this); timer_->setInterval(1000);
  connect(timer_, &QTimer::timeout, this, [this]() {refresh();});
  refresh(); timer_->start();
}
void MappingHealthPanel::refresh() {
  const QString path = QDir(directory_).filePath(QStringLiteral("health.json"));
  const auto report = QJsonDocument::fromJson(read_bounded(path)).object();
  if (directory_.isEmpty() || identity_.isEmpty() || report.value(QStringLiteral("session_id")).toString() != identity_) {
    summary_->setText(QStringLiteral("健康诊断：缺少本会话报告；地图质量未独立验收。"));
    details_ = QStringLiteral("请核对本会话 health.json 和 health_reporter 日志。诊断不读取或发送控制 socket。");
    return;
  }
  const auto stamp = QFileInfo(path).lastModified();
  const auto lifecycle = report.value(QStringLiteral("lifecycle")).toString();
  const bool final = lifecycle == "STOPPED" || lifecycle == "COMPLETE" || lifecycle == "PARTIAL" || lifecycle == "FAILED";
  const bool stale = !final && (!stamp.isValid() || stamp.msecsTo(QDateTime::currentDateTime()) > 5000);
  summary_->setText((stale ? QStringLiteral("诊断报告已停止更新；") :
    (final ? QStringLiteral("最终诊断快照：") : QStringLiteral("健康诊断："))) +
    report.value(QStringLiteral("summary_zh")).toString(QStringLiteral("地图质量未独立验收。")));
  details_ = QString::fromUtf8(read_bounded(QDir(directory_).filePath(QStringLiteral("diagnosis_zh.md"))));
  if (details_.isEmpty()) {details_ = QString::fromUtf8(QJsonDocument(report).toJson(QJsonDocument::Indented));}
}
}
