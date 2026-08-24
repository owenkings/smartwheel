#include "smartwheel_rviz_plugins/camera_panel.hpp"
#include "smartwheel_rviz_plugins/occupancy_grid_panel.hpp"
#include "smartwheel_rviz_plugins/teleop_panel.hpp"

#include <QApplication>
#include <QKeyEvent>
#include <QLabel>
#include <QTest>
#include <gtest/gtest.h>
#include <rviz_common/config.hpp>

namespace smartwheel_rviz_plugins
{
namespace
{

class TestableTeleopPanel : public TeleopPanel
{
public:
  bool filter(QEvent * event)
  {
    return eventFilter(this, event);
  }

  QString activeKeys() const
  {
    const auto * label = findChild<QLabel *>("teleopKeysLabel");
    return label ? label->text() : QString();
  }
};

}  // namespace

TEST(CameraPanelConfig, SavesAndLoadsIndependentInstanceSettings)
{
  rviz_common::Config input;
  input.mapSetValue("Name", "Rear Camera");
  input.mapSetValue("ImageTopic", "/camera/rear/image_raw");
  input.mapSetValue("CameraInfoTopic", "/camera/rear/camera_info");
  input.mapSetValue("TransportHint", "compressed");
  input.mapSetValue("PhysicalLabel", "Right-Side Camera");
  input.mapSetValue("KeepAspectRatio", false);
  input.mapSetValue("ShowTimestamp", false);
  input.mapSetValue("ShowFps", true);
  input.mapSetValue("Mirror", true);
  input.mapSetValue("RotationDegrees", 180);
  input.mapSetValue("MaxDisplayFps", 10.0);
  input.mapSetValue("OfflineThresholdSec", 4.0);
  input.mapSetValue("DetailsExpanded", false);

  CameraPanel panel;
  panel.load(input);
  rviz_common::Config output;
  panel.save(output);
  QString text;
  bool flag = false;
  int integer = 0;
  float number = 0.0F;
  ASSERT_TRUE(output.mapGetString("ImageTopic", &text));
  EXPECT_EQ(text, "/camera/rear/image_raw");
  ASSERT_TRUE(output.mapGetString("CameraInfoTopic", &text));
  EXPECT_EQ(text, "/camera/rear/camera_info");
  ASSERT_TRUE(output.mapGetString("TransportHint", &text));
  EXPECT_EQ(text, "compressed");
  ASSERT_TRUE(output.mapGetString("PhysicalLabel", &text));
  EXPECT_EQ(text, "Right-Side Camera");
  ASSERT_TRUE(output.mapGetBool("KeepAspectRatio", &flag));
  EXPECT_FALSE(flag);
  ASSERT_TRUE(output.mapGetBool("Mirror", &flag));
  EXPECT_TRUE(flag);
  ASSERT_TRUE(output.mapGetInt("RotationDegrees", &integer));
  EXPECT_EQ(integer, 180);
  ASSERT_TRUE(output.mapGetFloat("MaxDisplayFps", &number));
  EXPECT_FLOAT_EQ(number, 10.0F);
  ASSERT_TRUE(output.mapGetFloat("OfflineThresholdSec", &number));
  EXPECT_FLOAT_EQ(number, 4.0F);
  ASSERT_TRUE(output.mapGetBool("DetailsExpanded", &flag));
  EXPECT_FALSE(flag);
}

TEST(CameraPanelConfig, LegacyConfigWithoutTransportDefaultsToCompressed)
{
  rviz_common::Config input;
  input.mapSetValue("ImageTopic", "/camera/front/image_raw");
  input.mapSetValue("CameraInfoTopic", "/camera/front/camera_info");

  CameraPanel panel;
  panel.load(input);
  rviz_common::Config output;
  panel.save(output);

  QString transport;
  ASSERT_TRUE(output.mapGetString("TransportHint", &transport));
  EXPECT_EQ(transport, "compressed");
}

TEST(CameraPanelConfig, CompressedTopicSuffixIsAddedExactlyOnce)
{
  EXPECT_EQ(
    CameraPanel::compressedTopic("/camera/front/image_raw"),
    "/camera/front/image_raw/compressed");
  EXPECT_EQ(
    CameraPanel::compressedTopic("/camera/front/image_raw/compressed"),
    "/camera/front/image_raw/compressed");
}

TEST(OccupancyGridPanelConfig, SavesMapFirstDetailsWithoutInventingAProductPath)
{
  rviz_common::Config input;
  input.mapSetValue("MapTopic", "/map/live");
  input.mapSetValue("OdomTopic", "/odometry/filtered");
  input.mapSetValue("PathTopic", "/mapping/optimized_path");
  input.mapSetValue("ProductPath", "");
  input.mapSetValue("AutoFit", true);
  input.mapSetValue("ShowOrigin", true);
  input.mapSetValue("SettingsExpanded", false);

  OccupancyGridPanel panel;
  panel.load(input);
  rviz_common::Config output;
  panel.save(output);

  QString text;
  bool flag = true;
  ASSERT_TRUE(output.mapGetString("MapTopic", &text));
  EXPECT_EQ(text, "/map/live");
  ASSERT_TRUE(output.mapGetString("ProductPath", &text));
  EXPECT_TRUE(text.isEmpty());
  ASSERT_TRUE(output.mapGetBool("SettingsExpanded", &flag));
  EXPECT_FALSE(flag);
  ASSERT_TRUE(output.mapGetBool("AutoFit", &flag));
  EXPECT_TRUE(flag);
  ASSERT_TRUE(output.mapGetBool("ShowOrigin", &flag));
  EXPECT_TRUE(flag);
}

TEST(TeleopPanelInput, RemoteDesktopRepeatPairsRemainHeldUntilFinalRelease)
{
  TestableTeleopPanel panel;
  panel.show();
  QApplication::processEvents();

  QKeyEvent press(
    QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, QStringLiteral("w"), true, 1);
  QKeyEvent repeat_release(
    QEvent::KeyRelease, Qt::Key_W, Qt::NoModifier, QStringLiteral("w"), true, 1);
  ASSERT_TRUE(panel.filter(&press));
  EXPECT_EQ(panel.activeKeys(), "Keys W");
  ASSERT_TRUE(panel.filter(&repeat_release));
  QTest::qWait(60);
  EXPECT_EQ(panel.activeKeys(), "Keys W");

  QKeyEvent repeat_press(
    QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, QStringLiteral("w"), true, 1);
  ASSERT_TRUE(panel.filter(&repeat_press));
  QTest::qWait(80);
  EXPECT_EQ(panel.activeKeys(), "Keys W");

  QKeyEvent final_release(
    QEvent::KeyRelease, Qt::Key_W, Qt::NoModifier, QStringLiteral("w"), false, 1);
  ASSERT_TRUE(panel.filter(&final_release));
  QTest::qWait(150);
  EXPECT_EQ(panel.activeKeys(), "Keys NONE");
}

}  // namespace smartwheel_rviz_plugins

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
