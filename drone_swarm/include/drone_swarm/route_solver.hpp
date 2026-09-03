#pragma once

#include <cstddef>
#include <vector>

namespace drone_swarm
{

struct RoutePlan
{
  std::size_t drone_index;
  std::vector<std::size_t> target_indices;
  double total_cost;
};

struct RouteResourceCosts
{
  std::vector<std::vector<double>> starts;
  std::vector<std::vector<std::vector<double>>> legs;
  std::vector<std::vector<double>> returns;
  std::vector<double> maximum;
};

class RouteSolver
{
public:
  static std::vector<RoutePlan> solve(
    const std::vector<std::vector<double>> & drone_start_costs,
    const std::vector<std::vector<std::vector<double>>> & drone_target_costs,
    const std::vector<double> & route_base_costs = {},
    const std::vector<double> & max_route_travel_costs = {},
    const std::vector<std::vector<double>> & target_return_costs = {},
    bool use_all_drones = true,
    std::size_t max_improvement_passes = 50,
    const RouteResourceCosts & resource_costs = {});
};

}  // namespace drone_swarm
