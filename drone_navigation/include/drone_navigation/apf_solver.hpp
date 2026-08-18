#ifndef DRONE_NAVIGATION__APF_SOLVER_HPP_
#define DRONE_NAVIGATION__APF_SOLVER_HPP_

#include <limits>
#include <vector>

namespace drone_navigation
{

struct Vec3
{
  double x {0.0};
  double y {0.0};
  double z {0.0};
};

enum class FlightMode
{
  MULTICOPTER,
  FIXED_WING
};

struct ApfParameters
{
  double obstacle_influence_radius_m {70.0};
  double fw_avoid_trigger_distance_m {60.0};
  double mc_corridor_half_width_m {2.5};
  double fw_corridor_half_width_m {10.0};
  double lidar_vertical_half_fov_rad {0.2617993878};
  double mc_attractive_gain {1.0};
  double fw_attractive_gain {1.0};
  double mc_repulsive_gain {2.5};
  double fw_repulsive_gain {20.0};
  double repulsive_distance_power {2.0};
  double fw_max_avoid_yaw_rad {0.6108652382};
  double fw_max_avoid_pitch_rad {0.1745329252};
  double vertical_escape_pitch_gain {2.0};
  double clearance_radius_m {1.5};
  double mc_max_horizontal_speed_m_s {4.0};
  double mc_max_climb_speed_m_s {2.0};
  double clear_hold_time_s {2.0};
};

struct ApfResult
{
  Vec3 attractive;
  Vec3 repulsive;
  Vec3 safe_velocity;
  bool avoidance_active {false};
  bool center_blocked {false};
  bool vertical_escape {false};
  double nearest_path_obstacle_distance_m {std::numeric_limits<double>::infinity()};
};

class ApfSolver
{
public:
  explicit ApfSolver(ApfParameters parameters = {});

  void setParameters(const ApfParameters & parameters);
  void reset();

  ApfResult update(
    const Vec3 & desired_velocity_enu,
    const std::vector<Vec3> & obstacle_points_relative_enu,
    FlightMode mode,
    double time_s);

private:
  ApfParameters parameters_;
  bool avoidance_active_ {false};
  double clear_since_s_ {-1.0};
};

}  // namespace drone_navigation

#endif  // DRONE_NAVIGATION__APF_SOLVER_HPP_
