#include "smartwheel_rviz_plugins/diagnostic_model.hpp"
#include "smartwheel_rviz_plugins/occupancy_grid_model.hpp"
#include "smartwheel_rviz_plugins/safety_gate.hpp"
#include "smartwheel_rviz_plugins/teleop_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace smartwheel_rviz_plugins
{
namespace
{

nav_msgs::msg::OccupancyGrid makeGrid(double yaw = 0.0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 3;
  grid.info.height = 2;
  grid.info.resolution = 0.5F;
  grid.info.origin.position.x = 1.0;
  grid.info.origin.position.y = 2.0;
  grid.info.origin.orientation.z = std::sin(yaw * 0.5);
  grid.info.origin.orientation.w = std::cos(yaw * 0.5);
  grid.data = {-1, 0, 100, 25, 64, 65};
  return grid;
}

TEST(OccupancyGridModel, RejectsInvalidDataLength)
{
  auto grid = makeGrid();
  grid.data.pop_back();
  OccupancyGridModel model;
  std::string error;
  EXPECT_FALSE(model.setMap(grid, &error));
  EXPECT_FALSE(error.empty());
}

TEST(OccupancyGridModel, AppliesRowMajorColorsAndVerticalImageFlip)
{
  OccupancyGridModel model;
  ASSERT_TRUE(model.setMap(makeGrid()));
  EXPECT_EQ(model.image().pixelColor(0, 1), QColor(128, 128, 128));
  EXPECT_EQ(model.image().pixelColor(1, 1), QColor(255, 255, 255));
  EXPECT_EQ(model.image().pixelColor(2, 1), QColor(0, 0, 0));
  EXPECT_EQ(model.image().pixelColor(2, 0), QColor(0, 0, 0));
}

TEST(OccupancyGridModel, ProjectsWorldPointWithMapOrigin)
{
  OccupancyGridModel model;
  ASSERT_TRUE(model.setMap(makeGrid()));
  const auto point = model.worldToScene(1.5, 2.5);
  EXPECT_NEAR(point.x(), 1.0, 1e-9);
  EXPECT_NEAR(point.y(), 1.0, 1e-9);
}

TEST(OccupancyGridModel, ProjectsWorldPointWithRotatedOrigin)
{
  OccupancyGridModel model;
  ASSERT_TRUE(model.setMap(makeGrid(M_PI_2)));
  const auto point = model.worldToScene(0.5, 2.5);
  EXPECT_NEAR(point.x(), 1.0, 1e-9);
  EXPECT_NEAR(point.y(), 1.0, 1e-9);
}

TEST(TeleopModel, CombinesForwardAndLeft)
{
  TeleopModel model;
  model.setKey('W', true, 1.0);
  model.setKey('a', true, 1.0);
  const auto command = model.command(1.1);
  EXPECT_DOUBLE_EQ(command.first, 0.10);
  EXPECT_DOUBLE_EQ(command.second, 0.25);
  EXPECT_EQ(model.activeKeys(), "A+W");
}

TEST(TeleopModel, OppositeKeysCancel)
{
  TeleopModel model;
  model.setKey('w', true, 1.0);
  model.setKey('s', true, 1.0);
  const auto command = model.command(1.1);
  EXPECT_DOUBLE_EQ(command.first, 0.0);
}

TEST(TeleopModel, StopHasPriority)
{
  TeleopModel model;
  model.setKey('w', true, 1.0);
  model.setKey('a', true, 1.0);
  model.stop(1.1);
  EXPECT_EQ(model.command(1.2), std::make_pair(0.0, 0.0));
  EXPECT_FALSE(model.deadman());
}

TEST(TeleopModel, TimesOutWithoutHeartbeat)
{
  TeleopModel model;
  model.setTimeout(0.3);
  model.setKey('w', true, 1.0);
  EXPECT_EQ(model.command(1.31), std::make_pair(0.0, 0.0));
  EXPECT_TRUE(model.timedOut());
}

TEST(TeleopModel, HeartbeatKeepsHeldCommandFresh)
{
  TeleopModel model;
  model.setKey('w', true, 1.0);
  model.heartbeat(1.3);
  EXPECT_GT(model.command(1.4).first, 0.0);
}

TEST(TeleopModel, FocusLostStops)
{
  TeleopModel model;
  model.setKey('d', true, 1.0);
  model.focusLost(1.1);
  EXPECT_EQ(model.command(1.1), std::make_pair(0.0, 0.0));
}

TEST(TeleopModel, ReleaseStopsMouseEquivalent)
{
  TeleopModel model;
  model.setKey('s', true, 1.0);
  model.setKey('s', false, 1.1);
  EXPECT_EQ(model.command(1.1), std::make_pair(0.0, 0.0));
}

TEST(DiagnosticModel, MapsLevelsAndMockMode)
{
  EXPECT_EQ(diagnosticLevelToStatus(0, true), UiStatus::MOCK);
  EXPECT_EQ(diagnosticLevelToStatus(0, false), UiStatus::ONLINE);
  EXPECT_EQ(diagnosticLevelToStatus(1, true), UiStatus::WARNING);
  EXPECT_EQ(diagnosticLevelToStatus(2, true), UiStatus::ERROR);
  EXPECT_EQ(diagnosticLevelToStatus(0, true, true), UiStatus::DISABLED);
}

TEST(SafetyGate, ClampsTimesOutAndRejectsInvalidCommands)
{
  SafetyGate gate(0.35, 0.15, 0.25);
  EXPECT_TRUE(gate.accept(0.5, -0.8, 1.0));
  auto [linear, angular, reason] = gate.output(1.2);
  EXPECT_DOUBLE_EQ(linear, 0.15);
  EXPECT_DOUBLE_EQ(angular, -0.25);
  EXPECT_EQ(reason, "ACTIVE");
  EXPECT_EQ(std::get<2>(gate.output(1.36)), "COMMAND_TIMEOUT");
  EXPECT_FALSE(gate.accept(NAN, 0.0, 2.0));
  EXPECT_EQ(gate.invalidCount(), 1);
}

TEST(SafetyGate, EmergencyStopHasPriority)
{
  SafetyGate gate(0.35, 0.15, 0.25);
  ASSERT_TRUE(gate.accept(0.1, 0.2, 1.0));
  gate.setEmergencyStop(true);
  EXPECT_EQ(gate.output(1.0), std::make_tuple(0.0, 0.0, std::string("EMERGENCY_STOP")));
}

}  // namespace
}  // namespace smartwheel_rviz_plugins
