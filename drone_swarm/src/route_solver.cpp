#include "drone_swarm/route_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_swarm
{
namespace
{

constexpr double kImprovementTolerance = 1.0e-9;

void validateCosts(
  const std::vector<std::vector<double>> & drone_start_costs,
  const std::vector<std::vector<double>> & target_costs)
{
  if (drone_start_costs.empty()) {
    throw std::invalid_argument("route optimization requires at least one drone");
  }
  const std::size_t target_count = drone_start_costs.front().size();
  if (target_count == 0) return;
  for (const auto & row : drone_start_costs) {
    if (row.size() != target_count) {
      throw std::invalid_argument("drone start-cost rows must have equal length");
    }
    for (const double cost : row) {
      if (std::isnan(cost) || cost < 0.0) {
        throw std::invalid_argument("drone start costs must be non-negative or infinity");
      }
    }
  }
  if (target_costs.size() != target_count) {
    throw std::invalid_argument("target-cost matrix size does not match target count");
  }
  for (const auto & row : target_costs) {
    if (row.size() != target_count) {
      throw std::invalid_argument("target-cost matrix must be square");
    }
    for (const double cost : row) {
      if (!std::isfinite(cost) || cost < 0.0) {
        throw std::invalid_argument("target-to-target costs must be finite and non-negative");
      }
    }
  }
}

std::vector<std::size_t> minimumRowAssignment(
  const std::vector<std::vector<double>> & row_costs)
{
  if (row_costs.empty()) return {};
  const std::size_t row_count = row_costs.size();
  const std::size_t column_count = row_costs.front().size();
  if (row_count > column_count) {
    throw std::invalid_argument("minimumRowAssignment requires rows <= columns");
  }

  std::vector<double> row_potential(row_count + 1, 0.0);
  std::vector<double> column_potential(column_count + 1, 0.0);
  std::vector<std::size_t> matched_row(column_count + 1, 0);
  std::vector<std::size_t> previous_column(column_count + 1, 0);

  for (std::size_t row = 1; row <= row_count; ++row) {
    matched_row[0] = row;
    std::size_t current_column = 0;
    std::vector<double> minimum_reduced_cost(
      column_count + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(column_count + 1, false);
    do {
      used[current_column] = true;
      const std::size_t current_row = matched_row[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0;
      for (std::size_t column = 1; column <= column_count; ++column) {
        if (used[column]) continue;
        const double reduced_cost = row_costs[current_row - 1][column - 1] -
          row_potential[current_row] - column_potential[column];
        if (reduced_cost < minimum_reduced_cost[column]) {
          minimum_reduced_cost[column] = reduced_cost;
          previous_column[column] = current_column;
        }
        if (minimum_reduced_cost[column] < delta) {
          delta = minimum_reduced_cost[column];
          next_column = column;
        }
      }
      if (!std::isfinite(delta)) {
        throw std::runtime_error("no feasible drone-to-target seed assignment exists");
      }
      for (std::size_t column = 0; column <= column_count; ++column) {
        if (used[column]) {
          row_potential[matched_row[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum_reduced_cost[column] -= delta;
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

  std::vector<std::size_t> assignment(row_count, column_count);
  for (std::size_t column = 1; column <= column_count; ++column) {
    if (matched_row[column] != 0) assignment[matched_row[column] - 1] = column - 1;
  }
  for (std::size_t row = 0; row < row_count; ++row) {
    if (assignment[row] == column_count ||
      !std::isfinite(row_costs[row][assignment[row]]))
    {
      throw std::runtime_error("no feasible drone-to-target seed assignment exists");
    }
  }
  return assignment;
}

double routeCost(
  std::size_t drone,
  const std::vector<std::size_t> & route,
  const std::vector<std::vector<double>> & start_costs,
  const std::vector<std::vector<double>> & target_costs)
{
  if (route.empty()) return 0.0;
  double cost = start_costs[drone][route.front()];
  for (std::size_t index = 1; index < route.size(); ++index) {
    cost += target_costs[route[index - 1]][route[index]];
  }
  return cost;
}

double insertionDelta(
  std::size_t drone,
  const std::vector<std::size_t> & route,
  std::size_t position,
  std::size_t target,
  const std::vector<std::vector<double>> & start_costs,
  const std::vector<std::vector<double>> & target_costs)
{
  if (route.empty()) return start_costs[drone][target];
  if (position == 0) {
    return start_costs[drone][target] + target_costs[target][route.front()] -
           start_costs[drone][route.front()];
  }
  if (position == route.size()) return target_costs[route.back()][target];
  const auto previous = route[position - 1];
  const auto next = route[position];
  return target_costs[previous][target] + target_costs[target][next] -
         target_costs[previous][next];
}

double removalDelta(
  std::size_t drone,
  const std::vector<std::size_t> & route,
  std::size_t position,
  const std::vector<std::vector<double>> & start_costs,
  const std::vector<std::vector<double>> & target_costs)
{
  if (route.size() == 1) return -start_costs[drone][route.front()];
  if (position == 0) {
    return start_costs[drone][route[1]] - start_costs[drone][route[0]] -
           target_costs[route[0]][route[1]];
  }
  if (position + 1 == route.size()) {
    return -target_costs[route[position - 1]][route[position]];
  }
  return target_costs[route[position - 1]][route[position + 1]] -
         target_costs[route[position - 1]][route[position]] -
         target_costs[route[position]][route[position + 1]];
}

bool improveTwoOpt(
  std::size_t drone,
  std::vector<std::size_t> & route,
  const std::vector<std::vector<double>> & start_costs,
  const std::vector<std::vector<double>> & target_costs)
{
  double best_delta = 0.0;
  std::size_t best_begin = 0;
  std::size_t best_end = 0;
  for (std::size_t begin = 0; begin < route.size(); ++begin) {
    for (std::size_t end = begin + 1; end < route.size(); ++end) {
      const double old_front = begin == 0 ?
        start_costs[drone][route[begin]] :
        target_costs[route[begin - 1]][route[begin]];
      const double new_front = begin == 0 ?
        start_costs[drone][route[end]] :
        target_costs[route[begin - 1]][route[end]];
      double delta = new_front - old_front;
      if (end + 1 < route.size()) {
        delta += target_costs[route[begin]][route[end + 1]] -
          target_costs[route[end]][route[end + 1]];
      }
      if (delta < best_delta - kImprovementTolerance) {
        best_delta = delta;
        best_begin = begin;
        best_end = end;
      }
    }
  }
  if (best_delta >= -kImprovementTolerance) return false;
  std::reverse(route.begin() + best_begin, route.begin() + best_end + 1);
  return true;
}

bool improveRelocation(
  std::vector<std::vector<std::size_t>> & routes,
  bool keep_nonempty,
  const std::vector<std::vector<double>> & start_costs,
  const std::vector<std::vector<double>> & target_costs)
{
  double best_delta = 0.0;
  std::size_t best_source = 0;
  std::size_t best_source_position = 0;
  std::size_t best_destination = 0;
  std::size_t best_destination_position = 0;
  bool found = false;

  for (std::size_t source = 0; source < routes.size(); ++source) {
    if (keep_nonempty && routes[source].size() <= 1) continue;
    for (std::size_t source_position = 0;
      source_position < routes[source].size(); ++source_position)
    {
      const auto target = routes[source][source_position];
      const double removal = removalDelta(
        source, routes[source], source_position, start_costs, target_costs);
      for (std::size_t destination = 0; destination < routes.size(); ++destination) {
        if (destination == source || !std::isfinite(start_costs[destination][target])) continue;
        for (std::size_t destination_position = 0;
          destination_position <= routes[destination].size(); ++destination_position)
        {
          const double delta = removal + insertionDelta(
            destination, routes[destination], destination_position, target,
            start_costs, target_costs);
          if (delta < best_delta - kImprovementTolerance) {
            best_delta = delta;
            best_source = source;
            best_source_position = source_position;
            best_destination = destination;
            best_destination_position = destination_position;
            found = true;
          }
        }
      }
    }
  }

  if (!found) return false;
  const auto target = routes[best_source][best_source_position];
  routes[best_source].erase(routes[best_source].begin() + best_source_position);
  routes[best_destination].insert(
    routes[best_destination].begin() + best_destination_position, target);
  return true;
}

}  // namespace

std::vector<RoutePlan> RouteSolver::solve(
  const std::vector<std::vector<double>> & drone_start_costs,
  const std::vector<std::vector<double>> & target_costs,
  bool use_all_drones,
  std::size_t max_improvement_passes)
{
  validateCosts(drone_start_costs, target_costs);
  const std::size_t drone_count = drone_start_costs.size();
  const std::size_t target_count = drone_start_costs.front().size();
  if (target_count == 0) return {};

  std::vector<std::vector<std::size_t>> routes(drone_count);
  std::vector<bool> assigned(target_count, false);
  if (use_all_drones) {
    if (drone_count <= target_count) {
      const auto seeds = minimumRowAssignment(drone_start_costs);
      for (std::size_t drone = 0; drone < drone_count; ++drone) {
        routes[drone].push_back(seeds[drone]);
        assigned[seeds[drone]] = true;
      }
    } else {
      std::vector<std::vector<double>> target_drone_costs(
        target_count, std::vector<double>(drone_count));
      for (std::size_t target = 0; target < target_count; ++target) {
        for (std::size_t drone = 0; drone < drone_count; ++drone) {
          target_drone_costs[target][drone] = drone_start_costs[drone][target];
        }
      }
      const auto seeds = minimumRowAssignment(target_drone_costs);
      for (std::size_t target = 0; target < target_count; ++target) {
        routes[seeds[target]].push_back(target);
        assigned[target] = true;
      }
    }
  }

  std::size_t assigned_count = std::count(assigned.begin(), assigned.end(), true);
  while (assigned_count < target_count) {
    double best_delta = std::numeric_limits<double>::infinity();
    std::size_t best_target = target_count;
    std::size_t best_drone = drone_count;
    std::size_t best_position = 0;
    for (std::size_t target = 0; target < target_count; ++target) {
      if (assigned[target]) continue;
      for (std::size_t drone = 0; drone < drone_count; ++drone) {
        if (!std::isfinite(drone_start_costs[drone][target])) continue;
        for (std::size_t position = 0; position <= routes[drone].size(); ++position) {
          const double delta = insertionDelta(
            drone, routes[drone], position, target, drone_start_costs, target_costs);
          if (delta < best_delta) {
            best_delta = delta;
            best_target = target;
            best_drone = drone;
            best_position = position;
          }
        }
      }
    }
    if (best_target == target_count) {
      throw std::runtime_error("at least one target is infeasible for every available drone");
    }
    routes[best_drone].insert(routes[best_drone].begin() + best_position, best_target);
    assigned[best_target] = true;
    ++assigned_count;
  }

  const bool keep_nonempty = use_all_drones && target_count >= drone_count;
  for (std::size_t pass = 0; pass < max_improvement_passes; ++pass) {
    bool improved = false;
    for (std::size_t drone = 0; drone < drone_count; ++drone) {
      while (improveTwoOpt(drone, routes[drone], drone_start_costs, target_costs)) {
        improved = true;
      }
    }
    if (improveRelocation(routes, keep_nonempty, drone_start_costs, target_costs)) {
      improved = true;
    }
    if (!improved) break;
  }

  std::vector<RoutePlan> plans;
  for (std::size_t drone = 0; drone < drone_count; ++drone) {
    if (routes[drone].empty()) continue;
    plans.push_back({
      drone, routes[drone],
      routeCost(drone, routes[drone], drone_start_costs, target_costs)});
  }
  return plans;
}

}  // namespace drone_swarm
