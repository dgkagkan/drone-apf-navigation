#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "drone_navigation/apf_solver.hpp"

using drone_navigation::ApfParameters;
using drone_navigation::ApfSolver;
using drone_navigation::FlightMode;
using drone_navigation::Vec3;

TEST(ApfSolver, LeavesClearCommandUnchanged)
{
  ApfSolver solver;
  const auto result = solver.update({20.0, 0.0, 0.0}, {}, FlightMode::FIXED_WING, 0.0);

  EXPECT_FALSE(result.avoidance_active);
  EXPECT_DOUBLE_EQ(result.safe_velocity.x, 20.0);
  EXPECT_DOUBLE_EQ(result.safe_velocity.y, 0.0);
}

TEST(ApfSolver, DeflectsAwayFromObstacleOnTheLeft)
{
  ApfParameters parameters;
  parameters.fw_repulsive_gain = 20.0;
  ApfSolver solver(parameters);
  const std::vector<Vec3> obstacles{{20.0, 1.0, 0.0}, {21.0, 1.0, 0.0}};

  const auto result = solver.update(
    {20.0, 0.0, 0.0}, obstacles, FlightMode::FIXED_WING, 0.0);

  EXPECT_TRUE(result.avoidance_active);
  EXPECT_LT(result.safe_velocity.y, 0.0);
  EXPECT_NEAR(std::hypot(result.safe_velocity.x, result.safe_velocity.y), 20.0, 0.2);
}

TEST(ApfSolver, RespectsFixedWingMaximumYaw)
{
  ApfParameters parameters;
  parameters.fw_repulsive_gain = 1000.0;
  parameters.fw_max_avoid_yaw_rad = 10.0 * M_PI / 180.0;
  ApfSolver solver(parameters);
  const std::vector<Vec3> obstacles{{5.0, 1.0, 0.0}};

  const auto result = solver.update(
    {20.0, 0.0, 0.0}, obstacles, FlightMode::FIXED_WING, 0.0);

  EXPECT_LE(std::fabs(std::atan2(result.safe_velocity.y, result.safe_velocity.x)),
    parameters.fw_max_avoid_yaw_rad + 1e-6);
}
