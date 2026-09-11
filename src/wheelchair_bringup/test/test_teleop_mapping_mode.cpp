#include <QtTest/QtTest>
#include "wheelchair_bringup/teleop_panel.hpp"

class InspectPanel : public wheelchair_bringup::TeleopPanel {
public:
  using TeleopPanel::mappingDriveAllowed;
  using TeleopPanel::setDir;
  using TeleopPanel::stop;
  using TeleopPanel::requestMappingMode;
  using TeleopPanel::eventFilter;
  void status(const QString & value) {
    enable_push_mode_ = true; backend_mode_ = value; mode_received_.start();
  }
  void pending(bool value) { mode_pending_ = value; }
  void ack(bool value) { drive_ack_ = value; }
  double linear() const { return target_linear_; }
  void showModeControls() { mode_widget_->show(); }
};

class MappingModeTest : public QObject {
  Q_OBJECT
private slots:
  void legacyPanelUnchanged() {
    InspectPanel p; QVERIFY(p.mappingDriveAllowed());
    p.setDir(true, false, false, false, true); QVERIFY(p.linear() > 0);
  }
  void pushAndBadFeedbackBlockHeldKeys() {
    for (const auto & mode : {"push", "blocked", "drive_no_feedback", "disabled"}) {
      InspectPanel p; p.status(mode); QVERIFY(!p.mappingDriveAllowed());
      p.setDir(true, false, false, false, true); QCOMPARE(p.linear(), 0.0);
    }
  }
  void pendingAndUnacknowledgedBlock() {
    InspectPanel p; p.status("drive"); QVERIFY(p.mappingDriveAllowed());
    p.pending(true); QVERIFY(!p.mappingDriveAllowed());
    p.pending(false); p.ack(false); QVERIFY(!p.mappingDriveAllowed());
  }
  void staleStatusBlocks() {
    InspectPanel p; p.status("drive"); QTest::qWait(1050);
    QVERIFY(!p.mappingDriveAllowed());
  }
  void unavailableServiceClearsCommand() {
    InspectPanel p; p.status("drive"); p.setDir(true, false, false, false, true);
    p.requestMappingMode(true); QCOMPARE(p.linear(), 0.0);
    QVERIFY(!p.mappingDriveAllowed());
  }
  void stopAndApplicationDeactivateClearCommand() {
    InspectPanel p; p.setDir(true, false, false, false, true); p.stop();
    QCOMPARE(p.linear(), 0.0);
    p.setDir(true, false, false, false, true);
    QEvent event(QEvent::ApplicationDeactivate); p.eventFilter(&p, &event);
    QCOMPARE(p.linear(), 0.0);
  }
  void renderControls() {
    InspectPanel p; p.status("push"); p.showModeControls();
    p.resize(450, 530); p.show(); QTest::qWait(50);
    QVERIFY(p.grab().save("push_mode_panel.png"));
  }
};
QTEST_MAIN(MappingModeTest)
#include "test_teleop_mapping_mode.moc"
