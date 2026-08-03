#include "drone_navigation/apf_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace drone_navigation
{
namespace
{

constexpr std::size_t kYawBins = 31;
constexpr std::size_t kPitchBins = 17;
constexpr double kFrontSectorHalfAngleRad = 1.0471975512;

double norm(const Vec3 & value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

ApfParameters validated(ApfParameters parameters)
{
  parameters.obstacle_influence_radius_m =
    std::max(1.0, parameters.obstacle_influence_radius_m);
  parameters.fw_avoid_trigger_distance_m = std::clamp(
    parameters.fw_avoid_trigger_distance_m, 1.0,
    parameters.obstacle_influence_radius_m);
  parameters.mc_corridor_half_width_m =
    std::max(0.2, parameters.mc_corridor_half_width_m);
  parameters.fw_corridor_half_width_m = std::max(
    parameters.mc_corridor_half_width_m, parameters.fw_corridor_half_width_m);
  parameters.lidar_vertical_half_fov_rad = std::clamp(
    parameters.lidar_vertical_half_fov_rad, 0.0174532925, 0.7853981634);
  parameters.mc_attractive_gain = std::max(0.05, parameters.mc_attractive_gain);
  parameters.fw_attractive_gain = std::max(0.05, parameters.fw_attractive_gain);
  parameters.mc_repulsive_gain = std::max(0.0, parameters.mc_repulsive_gain);
  parameters.fw_repulsive_gain = std::max(0.0, parameters.fw_repulsive_gain);
  parameters.repulsive_distance_power =
    std::clamp(parameters.repulsive_distance_power, 0.25, 4.0);
  parameters.fw_max_avoid_yaw_rad = std::clamp(
    parameters.fw_max_avoid_yaw_rad, 0.0872664626, 1.0471975512);
  parameters.fw_max_avoid_pitch_rad = std::clamp(
    parameters.fw_max_avoid_pitch_rad, 0.0174532925,
    parameters.lidar_vertical_half_fov_rad - 0.0087266463);
  parameters.vertical_escape_pitch_gain = std::clamp(
    parameters.vertical_escape_pitch_gain, 1.0, 3.0);
  parameters.clearance_radius_m = std::max(0.1, parameters.clearance_radius_m);
  parameters.mc_max_horizontal_speed_m_s =
    std::max(0.2, parameters.mc_max_horizontal_speed_m_s);
  parameters.mc_max_climb_speed_m_s =
    std::max(0.2, parameters.mc_max_climb_speed_m_s);
  parameters.clear_hold_time_s = std::max(0.2, parameters.clear_hold_time_s);
  return parameters;
}

}  // namespace

ApfSolver::ApfSolver(ApfParameters parameters)
: parameters_(validated(parameters))
{
}

void ApfSolver::setParameters(const ApfParameters & parameters)
{
  parameters_ = validated(parameters);
  reset();
}

void ApfSolver::reset()
{
  avoidance_active_ = false;
  clear_since_s_ = -1.0;
}

ApfResult ApfSolver::update(
  const Vec3 & desired, const std::vector<Vec3> & obstacles,
  FlightMode mode, double time_s)
{
  ApfResult result;
  result.safe_velocity = desired;
  const double desired_speed = norm(desired);
  const double desired_horizontal = std::hypot(desired.x, desired.y);
  if (desired_speed < 0.05 || desired_horizontal < 0.05) {
    reset();
    return result;
  }

  const Vec3 forward{desired.x / desired_horizontal, desired.y / desired_horizontal, 0.0};
  const Vec3 left{-forward.y, forward.x, 0.0};
  const double attractive_gain = mode == FlightMode::FIXED_WING ?
    parameters_.fw_attractive_gain : parameters_.mc_attractive_gain;
  result.attractive = {
    attractive_gain * desired.x / desired_speed,
    attractive_gain * desired.y / desired_speed,
    attractive_gain * desired.z / desired_speed};

  constexpr std::size_t bin_count = kYawBins * kPitchBins;
  std::array<double, bin_count> bin_distance;
  std::array<Vec3, bin_count> bin_point{};
  bin_distance.fill(std::numeric_limits<double>::infinity());

  const double corridor = mode == FlightMode::FIXED_WING ?
    parameters_.fw_corridor_half_width_m : parameters_.mc_corridor_half_width_m;
  bool structure_ahead = false;
  double highest_center_elevation = -1.5707963268;

  for (const auto & point : obstacles) {
    const double distance = norm(point);
    if (!std::isfinite(distance) || distance < 0.3 ||
      distance > parameters_.obstacle_influence_radius_m)
    {
      continue;
    }

    const double along = point.x * forward.x + point.y * forward.y;
    const double signed_lateral = point.x * left.x + point.y * left.y;
    const double lateral = std::fabs(signed_lateral);
    const double horizontal_distance = std::hypot(along, lateral);
    const double yaw = std::atan2(signed_lateral, along);
    const double pitch = std::atan2(point.z, horizontal_distance);
    if (along <= 0.3 || std::fabs(yaw) >= kFrontSectorHalfAngleRad || lateral > corridor ||
      std::fabs(pitch) > parameters_.lidar_vertical_half_fov_rad)
    {
      continue;
    }

    structure_ahead = true;
    const double path_offset = std::hypot(signed_lateral, point.z);
    if (lateral <= parameters_.clearance_radius_m) {
      highest_center_elevation = std::max(highest_center_elevation, pitch);
    }
    if (path_offset <= parameters_.clearance_radius_m) {
      result.center_blocked = true;
      result.nearest_path_obstacle_distance_m = std::min(
        result.nearest_path_obstacle_distance_m, distance);
    }

    const double normalized_yaw =
      (yaw + kFrontSectorHalfAngleRad) / (2.0 * kFrontSectorHalfAngleRad);
    const double normalized_pitch =
      (pitch + parameters_.lidar_vertical_half_fov_rad) /
      (2.0 * parameters_.lidar_vertical_half_fov_rad);
    const auto yaw_bin = static_cast<std::size_t>(std::clamp(
      static_cast<int>(normalized_yaw * kYawBins), 0, static_cast<int>(kYawBins) - 1));
    const auto pitch_bin = static_cast<std::size_t>(std::clamp(
      static_cast<int>(normalized_pitch * kPitchBins), 0,
      static_cast<int>(kPitchBins) - 1));
    const auto bin = pitch_bin * kYawBins + yaw_bin;
    if (distance < bin_distance[bin]) {
      bin_distance[bin] = distance;
      bin_point[bin] = point;
    }
  }

  const double trigger = mode == FlightMode::FIXED_WING ?
    parameters_.fw_avoid_trigger_distance_m : parameters_.obstacle_influence_radius_m;
  const bool direct_path_blocked = result.nearest_path_obstacle_distance_m <= trigger;
  const bool obstacle_requires_avoidance =
    direct_path_blocked || (avoidance_active_ && structure_ahead);
  if (obstacle_requires_avoidance) {
    avoidance_active_ = true;
    clear_since_s_ = -1.0;
  } else if (avoidance_active_) {
    if (clear_since_s_ < 0.0) clear_since_s_ = time_s;
    if (time_s - clear_since_s_ >= parameters_.clear_hold_time_s) reset();
  }
  result.avoidance_active = avoidance_active_;
  if (!avoidance_active_) return result;

  const double repulsive_gain = mode == FlightMode::FIXED_WING ?
    parameters_.fw_repulsive_gain : parameters_.mc_repulsive_gain;
  std::size_t active_bins = 0;
  for (std::size_t index = 0; index < bin_count; ++index) {
    const double distance = bin_distance[index];
    if (!std::isfinite(distance)) continue;
    const double proximity = 1.0 - distance / parameters_.obstacle_influence_radius_m;
    const double magnitude =
      repulsive_gain * std::pow(proximity, parameters_.repulsive_distance_power);
    result.repulsive.x -= bin_point[index].x / distance * magnitude;
    result.repulsive.y -= bin_point[index].y / distance * magnitude;
    result.repulsive.z -= bin_point[index].z / distance * magnitude;
    ++active_bins;
  }
  if (active_bins > 0) {
    result.repulsive.x /= static_cast<double>(active_bins);
    result.repulsive.y /= static_cast<double>(active_bins);
    result.repulsive.z /= static_cast<double>(active_bins);
  }

  Vec3 candidate{
    result.attractive.x + result.repulsive.x,
    result.attractive.y + result.repulsive.y,
    result.attractive.z + result.repulsive.z};

  const double upper_limit = parameters_.lidar_vertical_half_fov_rad - 0.0349065850;
  const bool upper_escape_visible = highest_center_elevation < upper_limit;
  double vertical_escape_pitch = 0.0;
  if (result.center_blocked && upper_escape_visible &&
    std::isfinite(result.nearest_path_obstacle_distance_m))
  {
    const double layer_step =
      2.0 * parameters_.lidar_vertical_half_fov_rad /
      static_cast<double>(kPitchBins - 1);
    vertical_escape_pitch = parameters_.vertical_escape_pitch_gain * std::max(
      0.0, highest_center_elevation + 0.5 * layer_step +
      std::atan2(
        parameters_.clearance_radius_m,
        result.nearest_path_obstacle_distance_m));
    result.vertical_escape = vertical_escape_pitch > 0.0 &&
      vertical_escape_pitch <= parameters_.fw_max_avoid_pitch_rad;
  }

  if (mode == FlightMode::FIXED_WING) {
    const double forward_force = candidate.x * forward.x + candidate.y * forward.y;
    const double lateral_force = candidate.x * left.x + candidate.y * left.y;
    const double yaw = std::clamp(
      std::atan2(lateral_force, std::max(0.1, forward_force)),
      -parameters_.fw_max_avoid_yaw_rad, parameters_.fw_max_avoid_yaw_rad);
    const double horizontal_force = std::hypot(forward_force, lateral_force);
    if (result.vertical_escape) {
      candidate.z = std::max(candidate.z, horizontal_force * std::tan(vertical_escape_pitch));
    }
    const double pitch = std::clamp(
      std::atan2(candidate.z, std::max(0.1, horizontal_force)),
      -parameters_.fw_max_avoid_pitch_rad, parameters_.fw_max_avoid_pitch_rad);
    const double horizontal_speed = desired_speed * std::cos(pitch);
    result.safe_velocity = {
      horizontal_speed * (forward.x * std::cos(yaw) + left.x * std::sin(yaw)),
      horizontal_speed * (forward.y * std::cos(yaw) + left.y * std::sin(yaw)),
      desired_speed * std::sin(pitch)};
    return result;
  }

  const double candidate_norm = norm(candidate);
  if (candidate_norm > 1e-6) {
    candidate.x *= desired_speed / candidate_norm;
    candidate.y *= desired_speed / candidate_norm;
    candidate.z *= desired_speed / candidate_norm;
  }
  if (result.vertical_escape) {
    candidate.z = std::max(
      candidate.z,
      parameters_.mc_max_horizontal_speed_m_s * std::tan(vertical_escape_pitch));
  }
  const double candidate_horizontal = std::hypot(candidate.x, candidate.y);
  if (candidate_horizontal > parameters_.mc_max_horizontal_speed_m_s) {
    const double scale = parameters_.mc_max_horizontal_speed_m_s / candidate_horizontal;
    candidate.x *= scale;
    candidate.y *= scale;
  }
  candidate.z = std::clamp(
    candidate.z, -parameters_.mc_max_climb_speed_m_s,
    parameters_.mc_max_climb_speed_m_s);
  result.safe_velocity = candidate;
  return result;
}

}  // namespace drone_navigation
