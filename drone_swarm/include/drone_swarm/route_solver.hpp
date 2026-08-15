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

class RouteSolver
{
public:
  static std::vector<RoutePlan> solve(
    const std::vector<std::vector<double>> & drone_start_costs,
    const std::vector<std::vector<double>> & target_costs,
    bool use_all_drones = true,
    std::size_t max_improvement_passes = 50);
};

}  // namespace drone_swarm
