#include "smartwheel_rviz_plugins/camera_panel.hpp"

#include <QApplication>
#include <gtest/gtest.h>
#include <rviz_common/config.hpp>

namespace smartwheel_rviz_plugins
{

TEST(CameraPanelConfig, SavesAndLoadsIndependentInstanceSettings)
{
  rviz_common::Config input;
  input.mapSetValue("Name", "Rear Camera");
  input.mapSetValue("ImageTopic", "/camera/rear/image_raw");
  input.mapSetValue("CameraInfoTopic", "/camera/rear/camera_info");
  input.mapSetValue("KeepAspectRatio", false);
  input.mapSetValue("ShowTimestamp", false);
  input.mapSetValue("ShowFps", true);
  input.mapSetValue("Mirror", true);
  input.mapSetValue("RotationDegrees", 180);
  input.mapSetValue("MaxDisplayFps", 10.0);
  input.mapSetValue("OfflineThresholdSec", 4.0);

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
}

}  // namespace smartwheel_rviz_plugins

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
