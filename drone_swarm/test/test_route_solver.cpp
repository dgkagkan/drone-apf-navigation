#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "drone_swarm/route_solver.hpp"

namespace
{

std::vector<std::vector<std::vector<double>>> perDroneCosts(
  std::size_t drone_count,
  const std::vector<std::vector<double>> & shared_costs)
{
  return std::vector<std::vector<std::vector<double>>>(drone_count, shared_costs);
}

}  // namespace

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

  const auto routes = drone_swarm::RouteSolver::solve(
    starts, perDroneCosts(starts.size(), between));

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

  const auto routes = drone_swarm::RouteSolver::solve(
    starts, perDroneCosts(starts.size(), between));

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

  const auto routes = drone_swarm::RouteSolver::solve(
    starts, perDroneCosts(starts.size(), between));

  std::size_t assigned_count = 0;
  for (const auto & route : routes) assigned_count += route.target_indices.size();
  EXPECT_EQ(assigned_count, target_count);
}

TEST(RouteSolver, UsesDroneSpecificTravelTimeAndBatteryLimit)
{
  const std::vector<std::vector<double>> starts{{10.0, 20.0}, {4.0, 8.0}};
  const std::vector<std::vector<std::vector<double>>> legs{
    {{0.0, 10.0}, {10.0, 0.0}},
    {{0.0, 3.0}, {3.0, 0.0}},
  };
  const std::vector<double> base_costs{0.0, 5.0};
  const std::vector<double> maximum_travel{100.0, 20.0};
  const std::vector<std::vector<double>> return_costs{{5.0, 5.0}, {4.0, 20.0}};

  const auto routes = drone_swarm::RouteSolver::solve(
    starts, legs, base_costs, maximum_travel, return_costs);

  ASSERT_EQ(routes.size(), 2u);
  EXPECT_EQ(routes[0].drone_index, 0u);
  EXPECT_EQ(routes[1].drone_index, 1u);
  EXPECT_EQ(routes[0].target_indices, (std::vector<std::size_t>{1}));
  EXPECT_EQ(routes[1].target_indices, (std::vector<std::size_t>{0}));
}

TEST(RouteSolver, BalancesCompletionCostAcrossAvailableDrones)
{
  constexpr std::size_t drone_count = 3;
  constexpr std::size_t target_count = 9;
  std::vector<std::vector<double>> starts(
    drone_count, std::vector<double>(target_count, 1.0));
  std::vector<std::vector<std::vector<double>>> legs(
    drone_count,
    std::vector<std::vector<double>>(
      target_count, std::vector<double>(target_count, 1.0)));
  for (std::size_t target = 0; target < target_count; ++target) {
    for (std::size_t drone = 0; drone < drone_count; ++drone) {
      legs[drone][target][target] = 0.0;
    }
    starts[2][target] = 1.1;
  }
  for (auto & row : legs[2]) {
    for (double & cost : row) {
      if (cost > 0.0) cost = 1.1;
    }
  }

  const auto routes = drone_swarm::RouteSolver::solve(starts, legs);

  ASSERT_EQ(routes.size(), drone_count);
  const auto [minimum, maximum] = std::minmax_element(
    routes.begin(), routes.end(),
    [](const auto & left, const auto & right) {
      return left.target_indices.size() < right.target_indices.size();
    });
  EXPECT_LE(maximum->target_indices.size() - minimum->target_indices.size(), 1u);
  const auto [minimum_cost, maximum_cost] = std::minmax_element(
    routes.begin(), routes.end(),
    [](const auto & left, const auto & right) {return left.total_cost < right.total_cost;});
  EXPECT_LE(maximum_cost->total_cost - minimum_cost->total_cost, 0.31);
}

TEST(RouteSolver, RejectsATimeOptimalRouteThatExceedsEnergyBudget)
{
  const std::vector<std::vector<double>> starts{{2.0}, {5.0}};
  const std::vector<std::vector<std::vector<double>>> legs{
    {{0.0}},
    {{0.0}},
  };
  drone_swarm::RouteResourceCosts energy;
  energy.starts = {{8.0}, {3.0}};
  energy.legs = {{{0.0}}, {{0.0}}};
  energy.returns = {{5.0}, {3.0}};
  energy.maximum = {10.0, 10.0};

  const auto routes = drone_swarm::RouteSolver::solve(
    starts, legs, {}, {}, {}, true, 50, energy);

  ASSERT_EQ(routes.size(), 1u);
  EXPECT_EQ(routes.front().drone_index, 1u);
}
