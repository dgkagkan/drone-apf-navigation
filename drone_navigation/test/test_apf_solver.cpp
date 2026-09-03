#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "drone_navigation/apf_solver.hpp"

using drone_navigation::ApfParameters;
using drone_navigation::ApfSolver;
using drone_navigation::FlightMode;
using drone_navigation::Vec3;
using drone_navigation::angleInsideSector;
using drone_navigation::calculateActiveSector;
using drone_navigation::normalizeAngle;

namespace
{

constexpr double kDegreesToRadians = M_PI / 180.0;

Vec3 horizontalVector(double magnitude, double angle_degrees)
{
  const double angle = angle_degrees * kDegreesToRadians;
  return {magnitude * std::cos(angle), magnitude * std::sin(angle), 0.0};
}

ApfParameters fixedMarginParameters(double margin_degrees = 20.0)
{
  ApfParameters parameters;
  parameters.sector_margin_min_rad = margin_degrees * kDegreesToRadians;
  parameters.sector_margin_max_rad = margin_degrees * kDegreesToRadians;
  return parameters;
}

}  // namespace

TEST(ApfSolver, LeavesClearCommandUnchanged)
{
  ApfSolver solver;
  const auto result = solver.update(
    {20.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, 0.0, {},
    FlightMode::FIXED_WING, 0.0);

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
    {20.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, 0.0, obstacles,
    FlightMode::FIXED_WING, 0.0);

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
    {20.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, 0.0, obstacles,
    FlightMode::FIXED_WING, 0.0);

  EXPECT_LE(std::fabs(std::atan2(result.safe_velocity.y, result.safe_velocity.x)),
    parameters.fw_max_avoid_yaw_rad + 1e-6);
}

TEST(ApfSolver, StartsFixedWingAvoidanceForObstacleAtOneHundredFiftyMeters)
{
  ApfParameters parameters;
  parameters.obstacle_influence_radius_m = 200.0;
  parameters.fw_avoid_trigger_distance_m = 190.0;
  ApfSolver solver(parameters);

  const auto result = solver.update(
    {20.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, 0.0, {{150.0, 0.0, 0.0}},
    FlightMode::FIXED_WING, 0.0);

  EXPECT_TRUE(result.center_blocked);
  EXPECT_TRUE(result.avoidance_active);
  EXPECT_DOUBLE_EQ(result.nearest_path_obstacle_distance_m, 150.0);
}

TEST(ApfSolver, DefaultRangeIgnoresObstacleBeyondSeventyMeters)
{
  ApfSolver solver;

  const auto result = solver.update(
    {15.0, 0.0, 0.0}, {15.0, 0.0, 0.0}, 0.0, {{80.0, 0.0, 0.0}},
    FlightMode::FIXED_WING, 0.0);

  EXPECT_FALSE(result.center_blocked);
  EXPECT_FALSE(result.avoidance_active);
  EXPECT_FALSE(std::isfinite(result.nearest_path_obstacle_distance_m));
}

TEST(ActiveSector, CoversShortestRegionFromZeroToMinusSixtyDegrees)
{
  const auto sector = calculateActiveSector(
    horizontalVector(10.0, 0.0), horizontalVector(10.0, -60.0), 0.0,
    fixedMarginParameters());

  EXPECT_NEAR(sector.center_rad / kDegreesToRadians, -30.0, 1e-9);
  EXPECT_NEAR(sector.half_width_rad / kDegreesToRadians, 50.0, 1e-9);
  EXPECT_TRUE(angleInsideSector(-80.0 * kDegreesToRadians, sector));
  EXPECT_TRUE(angleInsideSector(20.0 * kDegreesToRadians, sector));
  EXPECT_FALSE(angleInsideSector(21.0 * kDegreesToRadians, sector));
}

TEST(ActiveSector, WrapsAcrossPlusMinusOneHundredEightyDegrees)
{
  const auto sector = calculateActiveSector(
    horizontalVector(10.0, 170.0), horizontalVector(10.0, -170.0), 0.0,
    fixedMarginParameters());

  EXPECT_NEAR(std::fabs(sector.center_rad), M_PI, 1e-9);
  EXPECT_NEAR(sector.half_width_rad / kDegreesToRadians, 30.0, 1e-9);
  EXPECT_TRUE(angleInsideSector(179.0 * kDegreesToRadians, sector));
  EXPECT_TRUE(angleInsideSector(-179.0 * kDegreesToRadians, sector));
  EXPECT_FALSE(angleInsideSector(0.0, sector));
}

TEST(ActiveSector, UsesMarginWhenCurrentEqualsDesired)
{
  const auto sector = calculateActiveSector(
    horizontalVector(10.0, 35.0), horizontalVector(10.0, 35.0), 0.0,
    fixedMarginParameters());

  EXPECT_NEAR(sector.center_rad / kDegreesToRadians, 35.0, 1e-9);
  EXPECT_NEAR(sector.half_width_rad / kDegreesToRadians, 20.0, 1e-9);
}

TEST(ActiveSector, InterpolatesAndClampsMarginFromCurrentSpeed)
{
  ApfParameters parameters;
  parameters.sector_margin_min_rad = 10.0 * kDegreesToRadians;
  parameters.sector_margin_max_rad = 30.0 * kDegreesToRadians;
  parameters.sector_margin_speed_min_m_s = 5.0;
  parameters.sector_margin_speed_max_m_s = 15.0;

  const auto below = calculateActiveSector(
    horizontalVector(2.0, 0.0), horizontalVector(10.0, 0.0), 0.0, parameters);
  const auto middle = calculateActiveSector(
    horizontalVector(10.0, 0.0), horizontalVector(10.0, 0.0), 0.0, parameters);
  const auto above = calculateActiveSector(
    horizontalVector(20.0, 0.0), horizontalVector(10.0, 0.0), 0.0, parameters);

  EXPECT_NEAR(below.margin_rad / kDegreesToRadians, 10.0, 1e-9);
  EXPECT_NEAR(middle.margin_rad / kDegreesToRadians, 20.0, 1e-9);
  EXPECT_NEAR(above.margin_rad / kDegreesToRadians, 30.0, 1e-9);
}

TEST(ActiveSector, FallsBackToCurrentDirectionForZeroDesiredVelocity)
{
  const auto sector = calculateActiveSector(
    horizontalVector(10.0, 45.0), {}, 0.0, fixedMarginParameters());

  EXPECT_NEAR(sector.desired_direction_rad / kDegreesToRadians, 45.0, 1e-9);
  EXPECT_TRUE(sector.desired_uses_fallback);
  EXPECT_FALSE(sector.current_uses_fallback);
}

TEST(ActiveSector, FallsBackToDesiredDirectionForZeroCurrentVelocity)
{
  const auto sector = calculateActiveSector(
    {}, horizontalVector(10.0, -25.0), 0.0, fixedMarginParameters());

  EXPECT_NEAR(sector.current_direction_rad / kDegreesToRadians, -25.0, 1e-9);
  EXPECT_TRUE(sector.current_uses_fallback);
  EXPECT_FALSE(sector.desired_uses_fallback);
}

TEST(ActiveSector, FallsBackToVehicleHeadingWhenBothVelocitiesAreZero)
{
  const double heading = 1.1;
  const auto sector = calculateActiveSector({}, {}, heading, fixedMarginParameters());

  EXPECT_NEAR(normalizeAngle(sector.current_direction_rad - heading), 0.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(sector.desired_direction_rad - heading), 0.0, 1e-12);
  EXPECT_TRUE(sector.current_uses_fallback);
  EXPECT_TRUE(sector.desired_uses_fallback);
}

TEST(ApfSolver, UsesObstacleInsideActiveSector)
{
  ApfSolver solver(fixedMarginParameters());
  const auto result = solver.update(
    horizontalVector(15.0, 0.0), horizontalVector(15.0, 0.0), 0.0,
    {horizontalVector(20.0, 15.0)}, FlightMode::FIXED_WING, 0.0);

  EXPECT_EQ(result.used_obstacles.size(), 1U);
  EXPECT_TRUE(result.sector_ignored_obstacles.empty());
}

TEST(ApfSolver, IgnoresObstacleOutsideActiveSector)
{
  ApfSolver solver(fixedMarginParameters());
  const auto result = solver.update(
    horizontalVector(15.0, 0.0), horizontalVector(15.0, 0.0), 0.0,
    {horizontalVector(20.0, 40.0)}, FlightMode::FIXED_WING, 0.0);

  EXPECT_TRUE(result.used_obstacles.empty());
  EXPECT_EQ(result.sector_ignored_obstacles.size(), 1U);
}

TEST(ApfSolver, UsesEmergencyObstacleOutsideActiveSector)
{
  auto parameters = fixedMarginParameters();
  parameters.emergency_radius_m = 5.0;
  ApfSolver solver(parameters);

  const auto result = solver.update(
    horizontalVector(15.0, 0.0), horizontalVector(15.0, 0.0), 0.0,
    {horizontalVector(3.0, 120.0)}, FlightMode::FIXED_WING, 0.0);

  EXPECT_EQ(result.used_obstacles.size(), 1U);
  EXPECT_TRUE(result.sector_ignored_obstacles.empty());
  EXPECT_TRUE(result.avoidance_active);
}
