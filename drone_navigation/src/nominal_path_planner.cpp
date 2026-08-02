#include "drone_navigation/nominal_path_planner.hpp"

#include <algorithm>
#include <cmath>

namespace drone_navigation
{
namespace
{

constexpr double pi = 3.14159265358979323846;
constexpr double gravity_m_s2 = 9.80665;

double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * pi);
}

}  // namespace

std::vector<NominalPathPoint> generateNominalPath(
  const NominalPathPoint & start,
  const NominalPathPoint & goal,
  const NominalPathParameters & parameters)
{
  const double speed_m_s = std::max(0.1, parameters.speed_m_s);
  const double max_bank_rad = std::clamp(
    std::fabs(parameters.max_bank_rad), 0.001, 0.5 * pi - 0.001);
  const double step_m = std::max(0.1, parameters.sample_distance_m);
  const double max_heading_step_rad =
    gravity_m_s2 * std::tan(max_bank_rad) * step_m / (speed_m_s * speed_m_s);
  const double max_altitude_step_m = step_m * std::tan(std::max(0.0, parameters.max_pitch_rad));

  std::vector<NominalPathPoint> path;
  path.reserve(parameters.max_points);
  path.push_back(start);
  NominalPathPoint current = start;

  for (std::size_t index = 1; index < parameters.max_points; ++index) {
    const double delta_x = goal.x - current.x;
    const double delta_y = goal.y - current.y;
    const double delta_z = goal.z - current.z;
    const double horizontal_distance_m = std::hypot(delta_x, delta_y);
    if (horizontal_distance_m <= parameters.arrival_radius_m &&
      std::fabs(delta_z) <= parameters.altitude_tolerance_m)
    {
      path.push_back(goal);
      break;
    }

    const double desired_heading_rad = std::atan2(delta_y, delta_x);
    const double heading_error_rad = normalizeAngle(
      desired_heading_rad - current.heading_enu_rad);
    current.heading_enu_rad = normalizeAngle(
      current.heading_enu_rad + std::clamp(
        heading_error_rad, -max_heading_step_rad, max_heading_step_rad));
    current.x += step_m * std::cos(current.heading_enu_rad);
    current.y += step_m * std::sin(current.heading_enu_rad);
    current.z += std::clamp(
      delta_z, -max_altitude_step_m, max_altitude_step_m);
    path.push_back(current);
  }

  return path;
}

}  // namespace drone_navigation
