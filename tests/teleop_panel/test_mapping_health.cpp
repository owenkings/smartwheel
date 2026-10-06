// SYNTHETIC local report reader. No ROS, control socket or device API.
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTest>
#include "wc_bringup/mapping_health.hpp"
class HealthTest : public QObject {
  Q_OBJECT
  void write(const QString & path, const QByteArray & bytes) {
    QFile file(path); QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
    QCOMPARE(file.write(bytes), qint64(bytes.size()));
  }
private slots:
  void reports_are_read_only_and_session_scoped() {
    QTemporaryDir directory; QVERIFY(directory.isValid());
    write(directory.filePath("view.rviz"), "# synthetic\n");
    write(directory.filePath("session.json"), "{\"session_id\":\"SYNTHETIC\"}");
    write(directory.filePath("health.json"), QJsonDocument(QJsonObject{{"session_id","SYNTHETIC"},
      {"summary_zh",QString::fromUtf8("地图质量未验证；左相机未枚举。")}}).toJson());
    write(directory.filePath("diagnosis_zh.md"), "SYNTHETIC observation and retest");
    wc_bringup::MappingHealthPanel panel({"mapping_rviz","-d",directory.filePath("view.rviz")});
    QVERIFY(panel.findChild<QLabel *>("mapping_health_summary")->text().contains(QString::fromUtf8("未枚举")));
    QVERIFY(!QFile::exists(directory.filePath("manual.sock")));
    write(directory.filePath("health.json"), "{\"session_id\":\"OTHER\",\"summary_zh\":\"do not display\"}");
    QTRY_VERIFY(!panel.findChild<QLabel *>("mapping_health_summary")->text().contains("do not display"));
    QTRY_VERIFY(panel.findChild<QLabel *>("mapping_health_summary")->text().contains(QString::fromUtf8("缺少本会话")));
    QVERIFY(!QFile::exists(directory.filePath("manual.sock")));
  }
  void missing_report_cannot_imply_healthy_map() {
    wc_bringup::MappingHealthPanel panel({"mapping_rviz"});
    QVERIFY(panel.findChild<QLabel *>("mapping_health_summary")->text().contains(QString::fromUtf8("未独立验收")));
    QVERIFY(panel.findChild<QPushButton *>("mapping_health_reasons") != nullptr);
  }
};
QTEST_MAIN(HealthTest)
#include "test_mapping_health.moc"
