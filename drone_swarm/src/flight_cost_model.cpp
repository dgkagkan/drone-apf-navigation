#include "drone_swarm/flight_cost_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_swarm
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double bearingNed(const FlightPoint & from, const FlightPoint & to)
{
  return std::atan2(to.x - from.x, to.y - from.y);
}

double angleDifference(double first, double second)
{
  return std::fabs(std::remainder(first - second, 2.0 * kPi));
}

}  // namespace

FlightCostModel::FlightCostModel(FlightCostParameters parameters)
: parameters_(parameters)
{
  if (parameters_.vertical_speed_m_s <= 0.0 ||
    parameters_.fixed_wing_turn_rate_rad_s <= 0.0 ||
    parameters_.fixed_wing_detour_factor < 1.0 ||
    parameters_.multicopter_detour_factor < 1.0 ||
    parameters_.transition_time_s < 0.0 ||
    parameters_.multicopter_power_w < 0.0 ||
    parameters_.fixed_wing_power_w < 0.0 ||
    parameters_.transition_power_w < 0.0 ||
    parameters_.climb_power_per_m_s_w < 0.0 ||
    parameters_.multicopter_speed_power_coefficient < 0.0 ||
    parameters_.fixed_wing_reference_speed_m_s <= 0.0)
  {
    throw std::invalid_argument("flight cost parameters are invalid");
  }
}

FlightEstimate FlightCostModel::estimateFromState(
  const FlightState & state,
  const FlightTarget & target) const
{
  return estimateSegment(
    state.position, state.heading_ned_rad, state.fixed_wing, target);
}

FlightEstimate FlightCostModel::estimateBetweenTargets(
  const FlightPoint & incoming_reference,
  const FlightTarget & from,
  const FlightTarget & to) const
{
  const double incoming_heading = bearingNed(incoming_reference, from.position);
  return estimateSegment(from.position, incoming_heading, from.fixed_wing, to);
}

FlightEstimate FlightCostModel::estimateSegment(
  const FlightPoint & from,
  double initial_heading_ned_rad,
  bool initial_fixed_wing,
  const FlightTarget & target) const
{
  const double speed_m_s = std::max(target.horizontal_speed_m_s, 0.2);
  const double horizontal_distance_m = std::hypot(
    target.position.x - from.x, target.position.y - from.y);
  const double vertical_distance_m = std::fabs(target.position.z - from.z);
  const double climb_distance_m = std::max(target.position.z - from.z, 0.0);
  const double transition_time_s = initial_fixed_wing == target.fixed_wing ?
    0.0 : parameters_.transition_time_s;

  double horizontal_time_s = 0.0;
  if (horizontal_distance_m > 1.0e-6) {
    const double detour_factor = target.fixed_wing ?
      parameters_.fixed_wing_detour_factor : parameters_.multicopter_detour_factor;
    horizontal_time_s = horizontal_distance_m * detour_factor / speed_m_s;
    if (target.fixed_wing) {
      const double target_bearing = bearingNed(from, target.position);
      horizontal_time_s += angleDifference(initial_heading_ned_rad, target_bearing) /
        parameters_.fixed_wing_turn_rate_rad_s;
    }
  }

  const double vertical_time_s = vertical_distance_m / parameters_.vertical_speed_m_s;
  const double travel_time_s = std::max(horizontal_time_s, vertical_time_s);
  const double total_time_s = transition_time_s + travel_time_s;

  double cruise_power_w = parameters_.multicopter_power_w +
    parameters_.multicopter_speed_power_coefficient * speed_m_s * speed_m_s;
  if (target.fixed_wing) {
    const double speed_ratio = std::clamp(
      speed_m_s / parameters_.fixed_wing_reference_speed_m_s, 0.5, 2.0);
    cruise_power_w = parameters_.fixed_wing_power_w *
      (0.7 + 0.3 * speed_ratio * speed_ratio * speed_ratio);
  }
  const double climb_speed_m_s = travel_time_s > 1.0e-6 ?
    climb_distance_m / travel_time_s : 0.0;
  cruise_power_w += parameters_.climb_power_per_m_s_w * climb_speed_m_s;

  FlightEstimate estimate;
  estimate.time_s = total_time_s;
  estimate.energy_wh =
    (travel_time_s * cruise_power_w +
    transition_time_s * parameters_.transition_power_w) / 3600.0;
  return estimate;
}

}  // namespace drone_swarm
