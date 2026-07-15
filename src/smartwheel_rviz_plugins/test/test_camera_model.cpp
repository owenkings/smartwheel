#include "smartwheel_rviz_plugins/camera_panel.hpp"

#include <gtest/gtest.h>

namespace smartwheel_rviz_plugins
{

TEST(CameraPanelModel, LimitsGuiFrameRate)
{
  EXPECT_TRUE(CameraPanel::frameAllowed(1.0, 0.0, 12.0));
  EXPECT_FALSE(CameraPanel::frameAllowed(1.04, 1.0, 12.0));
  EXPECT_TRUE(CameraPanel::frameAllowed(1.09, 1.0, 12.0));
}

TEST(CameraPanelModel, DetectsOfflineThreshold)
{
  EXPECT_TRUE(CameraPanel::isOffline(5.0, 0.0, 2.0));
  EXPECT_FALSE(CameraPanel::isOffline(5.0, 3.1, 2.0));
  EXPECT_TRUE(CameraPanel::isOffline(5.2, 3.1, 2.0));
}

}  // namespace smartwheel_rviz_plugins
