#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "drone_swarm/assignment_solver.hpp"

TEST(AssignmentSolver, FindsGlobalMinimumInsteadOfRepeatedNearestDrone)
{
  const std::vector<std::vector<double>> costs{
    {1.0, 2.0},
    {2.0, 100.0},
  };

  const auto assignments = drone_swarm::AssignmentSolver::solve(costs);

  ASSERT_EQ(assignments.size(), 2u);
  double total_cost = 0.0;
  for (const auto & assignment : assignments) total_cost += assignment.cost;
  EXPECT_DOUBLE_EQ(total_cost, 4.0);
}

TEST(AssignmentSolver, SelectsBestDroneSubsetForRectangularMatrix)
{
  const std::vector<std::vector<double>> costs{
    {10.0, 10.0},
    {1.0, 20.0},
    {20.0, 2.0},
  };

  const auto assignments = drone_swarm::AssignmentSolver::solve(costs);

  ASSERT_EQ(assignments.size(), 2u);
  double total_cost = 0.0;
  for (const auto & assignment : assignments) total_cost += assignment.cost;
  EXPECT_DOUBLE_EQ(total_cost, 3.0);
}

TEST(AssignmentSolver, RejectsTooFewDrones)
{
  const std::vector<std::vector<double>> costs{{1.0, 2.0}};
  EXPECT_THROW(drone_swarm::AssignmentSolver::solve(costs), std::invalid_argument);
}
