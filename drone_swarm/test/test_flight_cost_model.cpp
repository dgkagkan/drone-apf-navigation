#include <gtest/gtest.h>

#include <cmath>

#include "drone_swarm/flight_cost_model.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

drone_swarm::FlightTarget fixedWingTarget(double x, double y, double z, double speed)
{
  return {{x, y, z}, speed, true};
}

}  // namespace

TEST(FlightCostModel, PenalizesAFixedWingTargetBehindTheCurrentHeading)
{
  drone_swarm::FlightCostModel model({});
  const drone_swarm::FlightState state{{0.0, 0.0, 15.0}, 0.0, true};

  const auto ahead = model.estimateFromState(state, fixedWingTarget(0.0, 300.0, 15.0, 15.0));
  const auto behind = model.estimateFromState(state, fixedWingTarget(0.0, -300.0, 15.0, 15.0));

  EXPECT_NEAR(behind.time_s - ahead.time_s, kPi / (25.0 * kPi / 180.0), 1.0e-6);
}

TEST(FlightCostModel, FixedWingCruiseUsesLessEnergyThanMulticopterTravel)
{
  drone_swarm::FlightCostModel model({});
  const drone_swarm::FlightState state{{0.0, 0.0, 15.0}, 0.0, true};
  auto fixed_wing = fixedWingTarget(0.0, 300.0, 15.0, 15.0);
  auto multicopter = fixed_wing;
  multicopter.fixed_wing = false;

  const auto fixed_wing_cost = model.estimateFromState(state, fixed_wing);
  const auto multicopter_cost = model.estimateFromState(state, multicopter);

  EXPECT_LT(fixed_wing_cost.energy_wh, multicopter_cost.energy_wh);
}

TEST(FlightCostModel, IncludesTransitionAndClimbEnergy)
{
  drone_swarm::FlightCostModel model({});
  const drone_swarm::FlightState multicopter{{0.0, 0.0, 10.0}, 0.0, false};
  const auto level_target = fixedWingTarget(0.0, 300.0, 10.0, 15.0);
  const auto climb_target = fixedWingTarget(0.0, 300.0, 40.0, 15.0);

  const auto level = model.estimateFromState(multicopter, level_target);
  const auto climb = model.estimateFromState(multicopter, climb_target);

  EXPECT_GE(level.time_s, 8.0);
  EXPECT_GT(climb.energy_wh, level.energy_wh);
}
