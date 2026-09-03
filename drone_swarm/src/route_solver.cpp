#include "drone_swarm/route_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_swarm
{
namespace
{

constexpr double kTolerance = 1.0e-9;
using Matrix = std::vector<std::vector<double>>;
using CostCube = std::vector<Matrix>;

struct SolverCosts
{
  const Matrix & starts;
  const CostCube & legs;
  const std::vector<double> & bases;
  const std::vector<double> & maximum_travel;
  const Matrix & returns;
  const RouteResourceCosts * resources;
};

struct SolutionScore
{
  double maximum {0.0};
  double spread {0.0};
  double total {0.0};
};

bool hasResourceConstraint(const SolverCosts & costs)
{
  return costs.resources != nullptr && !costs.resources->starts.empty();
}

void validateValue(double value, const char * description, bool allow_infinity)
{
  if (std::isnan(value) || value < 0.0 || (!allow_infinity && !std::isfinite(value))) {
    throw std::invalid_argument(description);
  }
}

void validateCosts(const SolverCosts & costs)
{
  if (costs.starts.empty()) {
    throw std::invalid_argument("route optimization requires at least one drone");
  }
  const std::size_t drone_count = costs.starts.size();
  const std::size_t target_count = costs.starts.front().size();
  if (costs.legs.size() != drone_count || costs.bases.size() != drone_count ||
    costs.maximum_travel.size() != drone_count || costs.returns.size() != drone_count)
  {
    throw std::invalid_argument("per-drone route cost dimensions do not match drone count");
  }
  for (std::size_t drone = 0; drone < drone_count; ++drone) {
    if (costs.starts[drone].size() != target_count ||
      costs.legs[drone].size() != target_count ||
      costs.returns[drone].size() != target_count)
    {
      throw std::invalid_argument("per-drone route cost dimensions do not match target count");
    }
    validateValue(costs.bases[drone], "route base costs must be finite and non-negative", false);
    validateValue(
      costs.maximum_travel[drone],
      "maximum route travel costs must be non-negative or infinity", true);
    for (std::size_t from = 0; from < target_count; ++from) {
      validateValue(
        costs.starts[drone][from], "drone start costs must be non-negative or infinity", true);
      validateValue(
        costs.returns[drone][from], "target return costs must be non-negative or infinity", true);
      if (costs.legs[drone][from].size() != target_count) {
        throw std::invalid_argument("per-drone target cost matrix must be square");
      }
      for (const double value : costs.legs[drone][from]) {
        validateValue(value, "target leg costs must be finite and non-negative", false);
      }
    }
  }

  if (!hasResourceConstraint(costs)) return;
  const auto & resources = *costs.resources;
  if (resources.starts.size() != drone_count || resources.legs.size() != drone_count ||
    resources.returns.size() != drone_count || resources.maximum.size() != drone_count)
  {
    throw std::invalid_argument("route resource dimensions do not match drone count");
  }
  for (std::size_t drone = 0; drone < drone_count; ++drone) {
    if (resources.starts[drone].size() != target_count ||
      resources.legs[drone].size() != target_count ||
      resources.returns[drone].size() != target_count)
    {
      throw std::invalid_argument("route resource dimensions do not match target count");
    }
    validateValue(
      resources.maximum[drone], "maximum route resource must be non-negative or infinity", true);
    for (std::size_t from = 0; from < target_count; ++from) {
      validateValue(
        resources.starts[drone][from],
        "route start resources must be non-negative or infinity", true);
      validateValue(
        resources.returns[drone][from],
        "route return resources must be non-negative or infinity", true);
      if (resources.legs[drone][from].size() != target_count) {
        throw std::invalid_argument("per-drone route resource matrix must be square");
      }
      for (const double value : resources.legs[drone][from]) {
        validateValue(value, "route leg resources must be finite and non-negative", false);
      }
    }
  }
}

std::vector<std::size_t> minimumRowAssignment(const Matrix & row_costs)
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
    std::vector<double> reduced_minimum(
      column_count + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(column_count + 1, false);
    do {
      used[current_column] = true;
      const std::size_t current_row = matched_row[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0;
      for (std::size_t column = 1; column <= column_count; ++column) {
        if (used[column]) continue;
        const double reduced = row_costs[current_row - 1][column - 1] -
          row_potential[current_row] - column_potential[column];
        if (reduced < reduced_minimum[column]) {
          reduced_minimum[column] = reduced;
          previous_column[column] = current_column;
        }
        if (reduced_minimum[column] < delta) {
          delta = reduced_minimum[column];
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
          reduced_minimum[column] -= delta;
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

double travelCost(
  std::size_t drone, const std::vector<std::size_t> & route, const SolverCosts & costs)
{
  if (route.empty()) return 0.0;
  double result = costs.starts[drone][route.front()];
  for (std::size_t index = 1; index < route.size(); ++index) {
    result += costs.legs[drone][route[index - 1]][route[index]];
  }
  return result;
}

double objectiveCost(
  std::size_t drone, const std::vector<std::size_t> & route, const SolverCosts & costs)
{
  return route.empty() ? 0.0 : costs.bases[drone] + travelCost(drone, route, costs);
}

SolutionScore solutionScore(
  const std::vector<std::vector<std::size_t>> & routes,
  const SolverCosts & costs)
{
  SolutionScore score;
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t drone = 0; drone < routes.size(); ++drone) {
    if (routes[drone].empty()) continue;
    const double route_cost = objectiveCost(drone, routes[drone], costs);
    score.maximum = std::max(score.maximum, route_cost);
    minimum = std::min(minimum, route_cost);
    score.total += route_cost;
  }
  if (std::isfinite(minimum)) score.spread = score.maximum - minimum;
  return score;
}

bool scoreBetter(const SolutionScore & candidate, const SolutionScore & current)
{
  if (candidate.maximum < current.maximum - kTolerance) return true;
  if (candidate.maximum > current.maximum + kTolerance) return false;
  if (candidate.spread < current.spread - kTolerance) return true;
  if (candidate.spread > current.spread + kTolerance) return false;
  return candidate.total < current.total - kTolerance;
}

double resourceTravelCost(
  std::size_t drone, const std::vector<std::size_t> & route, const SolverCosts & costs)
{
  if (route.empty() || !hasResourceConstraint(costs)) return 0.0;
  const auto & resources = *costs.resources;
  double result = resources.starts[drone][route.front()];
  for (std::size_t index = 1; index < route.size(); ++index) {
    result += resources.legs[drone][route[index - 1]][route[index]];
  }
  return result;
}

bool routeFeasible(
  std::size_t drone, const std::vector<std::size_t> & route, const SolverCosts & costs)
{
  if (route.empty()) return true;
  const double travel = travelCost(drone, route, costs);
  const double return_cost = costs.returns[drone][route.back()];
  const bool travel_feasible = std::isfinite(travel) && std::isfinite(return_cost) &&
    travel + return_cost <= costs.maximum_travel[drone] + kTolerance;
  if (!travel_feasible || !hasResourceConstraint(costs)) return travel_feasible;

  const auto & resources = *costs.resources;
  const double resource = resourceTravelCost(drone, route, costs);
  const double return_resource = resources.returns[drone][route.back()];
  return std::isfinite(resource) && std::isfinite(return_resource) &&
         resource + return_resource <= resources.maximum[drone] + kTolerance;
}

bool improveTwoOpt(
  std::size_t drone, std::vector<std::size_t> & route, const SolverCosts & costs)
{
  double best_delta = 0.0;
  std::vector<std::size_t> best_route;
  for (std::size_t begin = 0; begin < route.size(); ++begin) {
    for (std::size_t end = begin + 1; end < route.size(); ++end) {
      auto candidate = route;
      std::reverse(candidate.begin() + begin, candidate.begin() + end + 1);
      if (!routeFeasible(drone, candidate, costs)) continue;
      const double delta = objectiveCost(drone, candidate, costs) -
        objectiveCost(drone, route, costs);
      if (delta < best_delta - kTolerance) {
        best_delta = delta;
        best_route = std::move(candidate);
      }
    }
  }
  if (best_route.empty()) return false;
  route = std::move(best_route);
  return true;
}

bool improveRelocation(
  std::vector<std::vector<std::size_t>> & routes,
  bool keep_nonempty, const SolverCosts & costs)
{
  const auto current_score = solutionScore(routes, costs);
  auto best_score = current_score;
  std::vector<std::vector<std::size_t>> best_routes;
  for (std::size_t source = 0; source < routes.size(); ++source) {
    if (keep_nonempty && routes[source].size() <= 1) continue;
    for (std::size_t source_position = 0;
      source_position < routes[source].size(); ++source_position)
    {
      const auto target = routes[source][source_position];
      auto source_route = routes[source];
      source_route.erase(source_route.begin() + source_position);
      if (!routeFeasible(source, source_route, costs)) continue;
      for (std::size_t destination = 0; destination < routes.size(); ++destination) {
        if (destination == source || !std::isfinite(costs.starts[destination][target])) continue;
        for (std::size_t destination_position = 0;
          destination_position <= routes[destination].size(); ++destination_position)
        {
          auto destination_route = routes[destination];
          destination_route.insert(
            destination_route.begin() + destination_position, target);
          if (!routeFeasible(destination, destination_route, costs)) continue;
          auto candidate_routes = routes;
          candidate_routes[source] = source_route;
          candidate_routes[destination] = std::move(destination_route);
          const auto candidate_score = solutionScore(candidate_routes, costs);
          if (scoreBetter(candidate_score, best_score)) {
            best_score = candidate_score;
            best_routes = std::move(candidate_routes);
          }
        }
      }
    }
  }
  if (best_routes.empty()) return false;
  routes = std::move(best_routes);
  return true;
}

}  // namespace

std::vector<RoutePlan> RouteSolver::solve(
  const Matrix & drone_start_costs,
  const CostCube & drone_target_costs,
  const std::vector<double> & route_base_costs,
  const std::vector<double> & max_route_travel_costs,
  const Matrix & target_return_costs,
  bool use_all_drones,
  std::size_t max_improvement_passes,
  const RouteResourceCosts & resource_costs)
{
  const std::size_t drone_count = drone_start_costs.size();
  if (drone_count == 0) {
    throw std::invalid_argument("route optimization requires at least one drone");
  }
  const std::size_t target_count = drone_start_costs.front().size();
  const std::vector<double> default_bases(drone_count, 0.0);
  const std::vector<double> default_limits(
    drone_count, std::numeric_limits<double>::infinity());
  const Matrix default_returns(drone_count, std::vector<double>(target_count, 0.0));
  const SolverCosts costs{
    drone_start_costs,
    drone_target_costs,
    route_base_costs.empty() ? default_bases : route_base_costs,
    max_route_travel_costs.empty() ? default_limits : max_route_travel_costs,
    target_return_costs.empty() ? default_returns : target_return_costs,
    &resource_costs};
  validateCosts(costs);
  if (target_count == 0) return {};

  Matrix seed_costs = drone_start_costs;
  for (std::size_t drone = 0; drone < drone_count; ++drone) {
    for (std::size_t target = 0; target < target_count; ++target) {
      const std::vector<std::size_t> seed{target};
      seed_costs[drone][target] = routeFeasible(drone, seed, costs) ?
        objectiveCost(drone, seed, costs) : std::numeric_limits<double>::infinity();
    }
  }

  std::vector<std::vector<std::size_t>> routes(drone_count);
  std::vector<bool> assigned(target_count, false);
  if (use_all_drones) {
    if (drone_count <= target_count) {
      const auto seeds = minimumRowAssignment(seed_costs);
      for (std::size_t drone = 0; drone < drone_count; ++drone) {
        routes[drone].push_back(seeds[drone]);
        assigned[seeds[drone]] = true;
      }
    } else {
      Matrix target_drone_costs(target_count, std::vector<double>(drone_count));
      for (std::size_t target = 0; target < target_count; ++target) {
        for (std::size_t drone = 0; drone < drone_count; ++drone) {
          target_drone_costs[target][drone] = seed_costs[drone][target];
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
    SolutionScore best_score{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
    std::size_t best_target = target_count;
    std::size_t best_drone = drone_count;
    std::size_t best_position = 0;
    for (std::size_t target = 0; target < target_count; ++target) {
      if (assigned[target]) continue;
      for (std::size_t drone = 0; drone < drone_count; ++drone) {
        if (!std::isfinite(costs.starts[drone][target])) continue;
        for (std::size_t position = 0; position <= routes[drone].size(); ++position) {
          auto candidate_route = routes[drone];
          candidate_route.insert(candidate_route.begin() + position, target);
          if (!routeFeasible(drone, candidate_route, costs)) continue;
          auto candidate_routes = routes;
          candidate_routes[drone] = std::move(candidate_route);
          const auto candidate_score = solutionScore(candidate_routes, costs);
          if (scoreBetter(candidate_score, best_score)) {
            best_score = candidate_score;
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
      while (improveTwoOpt(drone, routes[drone], costs)) improved = true;
    }
    if (improveRelocation(routes, keep_nonempty, costs)) improved = true;
    if (!improved) break;
  }

  std::vector<RoutePlan> plans;
  for (std::size_t drone = 0; drone < drone_count; ++drone) {
    if (routes[drone].empty()) continue;
    plans.push_back({drone, routes[drone], objectiveCost(drone, routes[drone], costs)});
  }
  return plans;
}

}  // namespace drone_swarm
