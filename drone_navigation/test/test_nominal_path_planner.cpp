#include <cmath>

#include <gtest/gtest.h>

#include "drone_navigation/nominal_path_planner.hpp"

using drone_navigation::NominalPathParameters;
using drone_navigation::NominalPathPoint;
using drone_navigation::generateNominalPath;

TEST(NominalPathPlanner, KeepsStraightGoalStraight)
{
  const auto path = generateNominalPath(
    NominalPathPoint{0.0, 0.0, 15.0, 0.0},
    NominalPathPoint{200.0, 0.0, 15.0, 0.0},
    NominalPathParameters{});

  ASSERT_GT(path.size(), 2U);
  EXPECT_NEAR(path[1].y, 0.0, 1e-6);
  EXPECT_NEAR(path.back().x, 200.0, 1e-6);
}

TEST(NominalPathPlanner, TurnsForwardTowardGoalBehindDrone)
{
  const auto path = generateNominalPath(
    NominalPathPoint{0.0, 0.0, 15.0, 0.0},
    NominalPathPoint{-500.0, 0.0, 15.0, 0.0},
    NominalPathParameters{});

  ASSERT_GT(path.size(), 20U);
  EXPECT_GT(path[1].x, 0.0);
  EXPECT_NE(std::fabs(path[10].y), 0.0);
  EXPECT_NEAR(path.back().x, -500.0, 10.0);
  EXPECT_NEAR(path.back().y, 0.0, 10.0);
}

TEST(NominalPathPlanner, RespectsMaximumHeadingChange)
{
  NominalPathParameters parameters;
  parameters.speed_m_s = 20.0;
  parameters.max_bank_rad = 0.7;
  parameters.sample_distance_m = 2.0;
  const auto path = generateNominalPath(
    NominalPathPoint{0.0, 0.0, 15.0, 0.0},
    NominalPathPoint{0.0, 200.0, 15.0, 0.0}, parameters);

  const double maximum_step = 9.80665 * std::tan(parameters.max_bank_rad) *
    parameters.sample_distance_m / (parameters.speed_m_s * parameters.speed_m_s);
  ASSERT_GT(path.size(), 2U);
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    const double heading_change = std::remainder(
      path[index].heading_enu_rad - path[index - 1].heading_enu_rad,
      2.0 * 3.14159265358979323846);
    EXPECT_LE(std::fabs(heading_change), maximum_step + 1e-9);
  }
}

TEST(NominalPathPlanner, HigherSpeedProducesAWiderTurn)
{
  NominalPathParameters slow_parameters;
  slow_parameters.speed_m_s = 10.0;
  NominalPathParameters fast_parameters = slow_parameters;
  fast_parameters.speed_m_s = 20.0;

  const NominalPathPoint start {0.0, 0.0, 15.0, 0.0};
  const NominalPathPoint goal {0.0, 200.0, 15.0, 0.0};
  const auto slow_path = generateNominalPath(start, goal, slow_parameters);
  const auto fast_path = generateNominalPath(start, goal, fast_parameters);

  ASSERT_GT(slow_path.size(), 10U);
  ASSERT_GT(fast_path.size(), 10U);
  EXPECT_GT(slow_path[10].y, fast_path[10].y);
}

TEST(NominalPathPlanner, InitialCourseDeterminesTheFirstTurnSide)
{
  const NominalPathParameters parameters;
  const NominalPathPoint goal {200.0, 0.0, 15.0, 0.0};
  const auto from_left = generateNominalPath(
    NominalPathPoint{0.0, 0.0, 15.0, 0.6}, goal, parameters);
  const auto from_right = generateNominalPath(
    NominalPathPoint{0.0, 0.0, 15.0, -0.6}, goal, parameters);

  ASSERT_GT(from_left.size(), 2U);
  ASSERT_GT(from_right.size(), 2U);
  EXPECT_GT(from_left[1].y, 0.0);
  EXPECT_LT(from_right[1].y, 0.0);
}
