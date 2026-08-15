#include "drone_swarm/assignment_solver.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_swarm
{

std::vector<Assignment> AssignmentSolver::solve(
  const std::vector<std::vector<double>> & drone_target_costs)
{
  if (drone_target_costs.empty()) return {};

  const std::size_t drone_count = drone_target_costs.size();
  const std::size_t target_count = drone_target_costs.front().size();
  if (target_count == 0) return {};
  if (drone_count < target_count) {
    throw std::invalid_argument("assignment requires at least as many drones as targets");
  }
  for (const auto & row : drone_target_costs) {
    if (row.size() != target_count) {
      throw std::invalid_argument("cost matrix rows must have equal length");
    }
    for (const double cost : row) {
      if (!std::isfinite(cost)) {
        throw std::invalid_argument("cost matrix contains a non-finite value");
      }
    }
  }

  // Hungarian algorithm with targets as rows and drones as columns. This
  // orientation naturally supports selecting a subset when drones > targets.
  std::vector<double> row_potential(target_count + 1, 0.0);
  std::vector<double> column_potential(drone_count + 1, 0.0);
  std::vector<std::size_t> matched_row(drone_count + 1, 0);
  std::vector<std::size_t> previous_column(drone_count + 1, 0);

  for (std::size_t target = 1; target <= target_count; ++target) {
    matched_row[0] = target;
    std::size_t current_column = 0;
    std::vector<double> minimum_reduced_cost(
      drone_count + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(drone_count + 1, false);

    do {
      used[current_column] = true;
      const std::size_t current_target = matched_row[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0;
      for (std::size_t drone = 1; drone <= drone_count; ++drone) {
        if (used[drone]) continue;
        const double reduced_cost = drone_target_costs[drone - 1][current_target - 1] -
          row_potential[current_target] - column_potential[drone];
        if (reduced_cost < minimum_reduced_cost[drone]) {
          minimum_reduced_cost[drone] = reduced_cost;
          previous_column[drone] = current_column;
        }
        if (minimum_reduced_cost[drone] < delta) {
          delta = minimum_reduced_cost[drone];
          next_column = drone;
        }
      }
      for (std::size_t drone = 0; drone <= drone_count; ++drone) {
        if (used[drone]) {
          row_potential[matched_row[drone]] += delta;
          column_potential[drone] -= delta;
        } else {
          minimum_reduced_cost[drone] -= delta;
        }
      }
      current_column = next_column;
    } while (matched_row[current_column] != 0);

    do {
      const std::size_t previous = previous_column[current_column];
      matched_row[current_column] = matched_row[previous];
      current_column = previous;
    } while (current_column != 0);
  }

  std::vector<Assignment> assignments;
  assignments.reserve(target_count);
  for (std::size_t drone = 1; drone <= drone_count; ++drone) {
    if (matched_row[drone] == 0) continue;
    const std::size_t target = matched_row[drone] - 1;
    assignments.push_back({drone - 1, target, drone_target_costs[drone - 1][target]});
  }
  return assignments;
}

}  // namespace drone_swarm
