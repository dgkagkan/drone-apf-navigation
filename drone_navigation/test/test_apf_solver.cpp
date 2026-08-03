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

TEST(ApfSolver, UsesSixtyDegreeFrontSector)
{
  constexpr double degrees_to_radians = M_PI / 180.0;
  constexpr double obstacle_distance_m = 1.5;
  const Vec3 inside_sector{
    obstacle_distance_m * std::cos(55.0 * degrees_to_radians),
    obstacle_distance_m * std::sin(55.0 * degrees_to_radians), 0.0};
  const Vec3 outside_sector{
    obstacle_distance_m * std::cos(65.0 * degrees_to_radians),
    obstacle_distance_m * std::sin(65.0 * degrees_to_radians), 0.0};
  ApfSolver inside_solver;
  ApfSolver outside_solver;

  const auto inside_result = inside_solver.update(
    {20.0, 0.0, 0.0}, {inside_sector}, FlightMode::FIXED_WING, 0.0);
  const auto outside_result = outside_solver.update(
    {20.0, 0.0, 0.0}, {outside_sector}, FlightMode::FIXED_WING, 0.0);

  EXPECT_TRUE(inside_result.avoidance_active);
  EXPECT_FALSE(outside_result.avoidance_active);
}
