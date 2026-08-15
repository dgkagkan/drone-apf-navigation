#pragma once

#include <cstddef>
#include <vector>

namespace drone_swarm
{

struct Assignment
{
  std::size_t drone_index;
  std::size_t target_index;
  double cost;
};

class AssignmentSolver
{
public:
  static std::vector<Assignment> solve(
    const std::vector<std::vector<double>> & drone_target_costs);
};

}  // namespace drone_swarm
