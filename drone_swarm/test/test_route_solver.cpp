#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "drone_swarm/route_solver.hpp"

TEST(RouteSolver, AssignsEveryTargetExactlyOnceWhenTargetsExceedDrones)
{
  const std::vector<std::vector<double>> starts{
    {1.0, 9.0, 10.0, 11.0},
    {11.0, 10.0, 9.0, 1.0},
  };
  const std::vector<std::vector<double>> between{
    {0.0, 1.0, 9.0, 10.0},
    {1.0, 0.0, 8.0, 9.0},
    {9.0, 8.0, 0.0, 1.0},
    {10.0, 9.0, 1.0, 0.0},
  };

  const auto routes = drone_swarm::RouteSolver::solve(starts, between);

  ASSERT_EQ(routes.size(), 2u);
  std::set<std::size_t> assigned;
  for (const auto & route : routes) {
    EXPECT_FALSE(route.target_indices.empty());
    assigned.insert(route.target_indices.begin(), route.target_indices.end());
  }
  EXPECT_EQ(assigned, (std::set<std::size_t>{0, 1, 2, 3}));
  EXPECT_DOUBLE_EQ(routes[0].total_cost + routes[1].total_cost, 4.0);
}

TEST(RouteSolver, SelectsDroneSubsetWhenThereAreMoreDronesThanTargets)
{
  const std::vector<std::vector<double>> starts{
    {10.0, 10.0},
    {1.0, 20.0},
    {20.0, 2.0},
  };
  const std::vector<std::vector<double>> between{{0.0, 5.0}, {5.0, 0.0}};

  const auto routes = drone_swarm::RouteSolver::solve(starts, between);

  ASSERT_EQ(routes.size(), 2u);
  EXPECT_EQ(routes[0].drone_index, 1u);
  EXPECT_EQ(routes[1].drone_index, 2u);
  EXPECT_DOUBLE_EQ(routes[0].total_cost + routes[1].total_cost, 3.0);
}

TEST(RouteSolver, AcceptsLargeTargetSetsWithoutAHardLimit)
{
  constexpr std::size_t target_count = 50;
  std::vector<std::vector<double>> starts(
    3, std::vector<double>(target_count, 0.0));
  std::vector<std::vector<double>> between(
    target_count, std::vector<double>(target_count, 0.0));
  for (std::size_t drone = 0; drone < starts.size(); ++drone) {
    for (std::size_t target = 0; target < target_count; ++target) {
      starts[drone][target] = std::fabs(
        static_cast<double>(target) - static_cast<double>(drone * 20));
    }
  }
  for (std::size_t from = 0; from < target_count; ++from) {
    for (std::size_t to = 0; to < target_count; ++to) {
      between[from][to] = std::fabs(
        static_cast<double>(from) - static_cast<double>(to));
    }
  }

  const auto routes = drone_swarm::RouteSolver::solve(starts, between);

  std::size_t assigned_count = 0;
  for (const auto & route : routes) assigned_count += route.target_indices.size();
  EXPECT_EQ(assigned_count, target_count);
}
