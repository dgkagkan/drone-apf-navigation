#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <drone_interfaces/msg/apf_telemetry.hpp>
#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <drone_interfaces/srv/set_apf_mode.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "drone_navigation/apf_solver.hpp"

using ApfTelemetry = drone_interfaces::msg::ApfTelemetry;
using MotionCommand = drone_interfaces::msg::MotionCommand;
using SetApfMode = drone_interfaces::srv::SetApfMode;
using SetBool = std_srvs::srv::SetBool;
using VehicleState = drone_interfaces::msg::VehicleState;
using drone_navigation::ApfParameters;
using drone_navigation::ApfSolver;
using drone_navigation::FlightMode;
using drone_navigation::Vec3;
using drone_navigation::calculateActiveSector;
using std::placeholders::_1;
using std::placeholders::_2;

class ApfSafetyNode : public rclcpp::Node
{
public:
  ApfSafetyNode()
  : Node("apf_safety"), base_parameters_(readParameters()), solver_(base_parameters_)
  {
    command_timeout_s_ = std::max(
      0.1, declare_parameter<double>("command_timeout_s", 0.5));
    obstacle_timeout_s_ = std::max(
      0.1, declare_parameter<double>("obstacle_timeout_s", 0.5));
    avoidance_enabled_ = declare_parameter<bool>("avoidance_enabled", true);
    const double debug_cloud_rate_hz = std::max(
      0.1, declare_parameter<double>("debug_cloud_publish_rate_hz", 5.0));
    debug_cloud_publish_period_s_ = 1.0 / debug_cloud_rate_hz;
    profiles_.emplace("stable", readProfileParameters("stable", base_parameters_));
    profiles_.emplace("normal", readProfileParameters("normal", base_parameters_));
    profiles_.emplace("sport", readProfileParameters("sport", base_parameters_));
    active_mode_ = declare_parameter<std::string>("default_mode", "normal");
    const auto initial_profile = profiles_.find(active_mode_);
    if (initial_profile == profiles_.end()) {
      RCLCPP_WARN(
        get_logger(), "Unknown default APF mode '%s'; using normal",
        active_mode_.c_str());
      active_mode_ = "normal";
    }
    active_lidar_range_m_ = profiles_.at(active_mode_).obstacle_influence_radius_m;
    applyActiveParameters();

    selected_sub_ = create_subscription<MotionCommand>(
      "/motion/selected_intent", 10, std::bind(&ApfSafetyNode::onCommand, this, _1));
    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&ApfSafetyNode::onState, this, _1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/perception/obstacles", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ApfSafetyNode::onObstacles, this, _1));
    lidar_range_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/perception/lidar_range_override",
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ApfSafetyNode::onLidarRange, this, _1));
    enabled_service_ = create_service<SetBool>(
      "/apf/set_enabled", std::bind(&ApfSafetyNode::setEnabled, this, _1, _2));
    mode_service_ = create_service<SetApfMode>(
      "/apf/set_mode", std::bind(&ApfSafetyNode::setMode, this, _1, _2));
    safe_pub_ = create_publisher<MotionCommand>("/motion/safe_command", 10);
    telemetry_pub_ = create_publisher<ApfTelemetry>(
      "/apf/telemetry", rclcpp::QoS(10).reliable().transient_local());
    used_obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/apf/obstacles_used", rclcpp::SensorDataQoS().keep_last(1));
    ignored_obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/apf/obstacles_sector_ignored", rclcpp::SensorDataQoS().keep_last(1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&ApfSafetyNode::onTimer, this));
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ApfSafetyNode::onParametersSet, this, std::placeholders::_1));

    publishModeTelemetry();
    RCLCPP_INFO(get_logger(), "APF safety ready in '%s' mode", active_mode_.c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    ApfParameters updated;
    double updated_lidar_range = 0.0;
    bool updated_avoidance_enabled = true;
    double updated_obstacle_timeout = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      updated = active_parameters_;
      updated_lidar_range = active_lidar_range_m_;
      updated_avoidance_enabled = avoidance_enabled_;
      updated_obstacle_timeout = obstacle_timeout_s_;

      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "obstacle_influence_radius") {
          updated_lidar_range = parameter.as_double();
          updated.obstacle_influence_radius_m = updated_lidar_range;
        } else if (name == "fw_avoid_trigger_dist") {
          updated.fw_avoid_trigger_distance_m = parameter.as_double();
        } else if (name == "mc_attractive_gain") {
          updated.mc_attractive_gain = parameter.as_double();
        } else if (name == "fw_attractive_gain") {
          updated.fw_attractive_gain = parameter.as_double();
        } else if (name == "mc_repulsive_gain") {
          updated.mc_repulsive_gain = parameter.as_double();
        } else if (name == "fw_repulsive_gain") {
          updated.fw_repulsive_gain = parameter.as_double();
        } else if (name == "repulsive_distance_power") {
          updated.repulsive_distance_power = parameter.as_double();
        } else if (name == "fw_max_avoid_angle_deg") {
          updated.fw_max_avoid_yaw_rad = parameter.as_double() * 0.017453292519943295;
        } else if (name == "fw_max_avoid_pitch_deg") {
          updated.fw_max_avoid_pitch_rad = parameter.as_double() * 0.017453292519943295;
        } else if (name == "vertical_escape_pitch_gain") {
          updated.vertical_escape_pitch_gain = parameter.as_double();
        } else if (name == "apf_clearance_radius") {
          updated.clearance_radius_m = parameter.as_double();
        } else if (name == "mc_speed") {
          updated.mc_max_horizontal_speed_m_s = parameter.as_double();
        } else if (name == "mc_climb_speed") {
          updated.mc_max_climb_speed_m_s = parameter.as_double();
        } else if (name == "avoidance_clear_hold_time") {
          updated.clear_hold_time_s = parameter.as_double();
        } else if (name == "sector_margin_min_deg") {
          updated.sector_margin_min_rad = parameter.as_double() * 0.017453292519943295;
        } else if (name == "sector_margin_max_deg") {
          updated.sector_margin_max_rad = parameter.as_double() * 0.017453292519943295;
        } else if (name == "sector_margin_speed_min") {
          updated.sector_margin_speed_min_m_s = parameter.as_double();
        } else if (name == "sector_margin_speed_max") {
          updated.sector_margin_speed_max_m_s = parameter.as_double();
        } else if (name == "sector_direction_min_speed") {
          updated.direction_min_speed_m_s = parameter.as_double();
        } else if (name == "emergency_radius") {
          updated.emergency_radius_m = parameter.as_double();
        } else if (name == "avoidance_enabled") {
          updated_avoidance_enabled = parameter.as_bool();
        } else if (name == "obstacle_timeout_s") {
          updated_obstacle_timeout = parameter.as_double();
        }
      }

      if (!std::isfinite(updated_lidar_range) || updated_lidar_range < 1.0 ||
        updated_lidar_range > 300.0)
      {
        result.successful = false;
        result.reason = "obstacle influence radius must be between 1 and 300 m";
      } else if (!std::isfinite(updated.fw_avoid_trigger_distance_m) ||
        updated.fw_avoid_trigger_distance_m < 1.0 ||
        updated.fw_avoid_trigger_distance_m > updated_lidar_range)
      {
        result.successful = false;
        result.reason = "fixed-wing avoidance trigger must be within the APF radius";
      } else if (!std::isfinite(updated.sector_margin_min_rad) ||
        !std::isfinite(updated.sector_margin_max_rad) ||
        updated.sector_margin_min_rad < 0.0 ||
        updated.sector_margin_max_rad < updated.sector_margin_min_rad ||
        updated.sector_margin_max_rad > 3.14159265358979323846)
      {
        result.successful = false;
        result.reason = "sector margins must be ordered and within 0-180 degrees";
      } else if (!std::isfinite(updated.sector_margin_speed_min_m_s) ||
        !std::isfinite(updated.sector_margin_speed_max_m_s) ||
        updated.sector_margin_speed_min_m_s < 0.0 ||
        updated.sector_margin_speed_max_m_s <= updated.sector_margin_speed_min_m_s)
      {
        result.successful = false;
        result.reason = "sector margin speed limits must be ordered";
      } else if (!std::isfinite(updated_obstacle_timeout) || updated_obstacle_timeout < 0.1 ||
        updated_obstacle_timeout > 10.0)
      {
        result.successful = false;
        result.reason = "obstacle timeout must be between 0.1 and 10 seconds";
      }

      if (result.successful) {
        updated.obstacle_influence_radius_m = updated_lidar_range;
        profiles_[active_mode_] = updated;
        active_parameters_ = updated;
        active_lidar_range_m_ = updated_lidar_range;
        avoidance_enabled_ = updated_avoidance_enabled;
        obstacle_timeout_s_ = updated_obstacle_timeout;
        solver_.setParameters(active_parameters_);
        if (!avoidance_enabled_) solver_.reset();
      }
    }
    if (result.successful) {
      RCLCPP_INFO(get_logger(), "Runtime APF parameter update accepted");
    }
    return result;
  }

  void setEnabled(
    const SetBool::Request::SharedPtr request,
    const SetBool::Response::SharedPtr response)
  {
    const bool enabled = request->data;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      avoidance_enabled_ = enabled;
      if (!avoidance_enabled_) solver_.reset();
    }
    try {
      set_parameter(rclcpp::Parameter("avoidance_enabled", enabled));
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Could not synchronize avoidance_enabled parameter: %s", exception.what());
    }
    response->success = true;
    response->message = enabled ? "APF enabled" : "APF disabled";
    publishModeTelemetry();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void setMode(
    const SetApfMode::Request::SharedPtr request,
    const SetApfMode::Response::SharedPtr response)
  {
    std::string requested_mode = request->mode;
    std::transform(
      requested_mode.begin(), requested_mode.end(), requested_mode.begin(),
      [](unsigned char character) {return static_cast<char>(std::tolower(character));});

    std::lock_guard<std::mutex> lock(mutex_);
    response->active_mode = active_mode_;
    const auto profile = profiles_.find(requested_mode);
    if (profile == profiles_.end()) {
      response->accepted = false;
      response->message =
        "unknown APF mode '" + request->mode + "'; expected stable, normal, or sport";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }
    if (requested_mode == active_mode_) {
      response->accepted = false;
      response->message = "APF mode '" + active_mode_ + "' is already active";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      return;
    }

    active_mode_ = requested_mode;
    applyActiveParameters();
    response->accepted = true;
    response->active_mode = active_mode_;
    response->message = "APF mode changed to '" + active_mode_ + "'";
    publishModeTelemetry();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  ApfParameters readParameters()
  {
    constexpr double degrees_to_radians = 0.017453292519943295;
    ApfParameters parameters;
    parameters.obstacle_influence_radius_m =
      declare_parameter<double>("obstacle_influence_radius", 70.0);
    parameters.fw_avoid_trigger_distance_m =
      declare_parameter<double>("fw_avoid_trigger_dist", 60.0);
    parameters.mc_corridor_half_width_m =
      declare_parameter<double>("mc_trail_half_width", 2.5);
    parameters.fw_corridor_half_width_m =
      declare_parameter<double>("fw_trail_half_width", 11.66155241746725);
    parameters.lidar_vertical_half_fov_rad =
      declare_parameter<double>("lidar_vertical_half_fov_deg", 15.0) * degrees_to_radians;
    parameters.mc_attractive_gain = declare_parameter<double>("mc_attractive_gain", 1.0);
    parameters.fw_attractive_gain =
      declare_parameter<double>("fw_attractive_gain", 0.510689137310558);
    parameters.mc_repulsive_gain = declare_parameter<double>("mc_repulsive_gain", 2.5);
    parameters.fw_repulsive_gain =
      declare_parameter<double>("fw_repulsive_gain", 10.414042935641696);
    parameters.repulsive_distance_power =
      declare_parameter<double>("repulsive_distance_power", 1.054655071200557);
    parameters.fw_max_avoid_yaw_rad =
      declare_parameter<double>("fw_max_avoid_angle_deg", 16.805443865619306) *
      degrees_to_radians;
    parameters.fw_max_avoid_pitch_rad =
      declare_parameter<double>("fw_max_avoid_pitch_deg", 13.988346309184921) *
      degrees_to_radians;
    parameters.vertical_escape_pitch_gain =
      declare_parameter<double>("vertical_escape_pitch_gain", 1.16097076371247);
    parameters.clearance_radius_m = declare_parameter<double>("apf_clearance_radius", 1.5);
    parameters.mc_max_horizontal_speed_m_s = declare_parameter<double>("mc_speed", 4.0);
    parameters.mc_max_climb_speed_m_s = declare_parameter<double>("mc_climb_speed", 2.0);
    parameters.clear_hold_time_s =
      declare_parameter<double>("avoidance_clear_hold_time", 2.0);
    parameters.sector_margin_min_rad =
      declare_parameter<double>("sector_margin_min_deg", 15.0) * degrees_to_radians;
    parameters.sector_margin_max_rad =
      declare_parameter<double>("sector_margin_max_deg", 35.0) * degrees_to_radians;
    parameters.sector_margin_speed_min_m_s =
      declare_parameter<double>("sector_margin_speed_min", 3.0);
    parameters.sector_margin_speed_max_m_s =
      declare_parameter<double>("sector_margin_speed_max", 20.0);
    parameters.direction_min_speed_m_s =
      declare_parameter<double>("sector_direction_min_speed", 0.5);
    parameters.emergency_radius_m =
      declare_parameter<double>("emergency_radius", 5.0);
    return parameters;
  }

  void applyActiveParameters()
  {
    auto parameters = profiles_.at(active_mode_);
    parameters.obstacle_influence_radius_m = active_lidar_range_m_;
    active_parameters_ = parameters;
    solver_.setParameters(parameters);
  }

  void onLidarRange(const std_msgs::msg::Float64::SharedPtr message)
  {
    constexpr double min_range_m = 70.0;
    constexpr double max_range_m = 300.0;
    if (!std::isfinite(message->data) || message->data < min_range_m ||
      message->data > max_range_m)
    {
      RCLCPP_WARN(
        get_logger(), "Rejected APF LiDAR range %.1f m; expected %.1f-%.1f m",
        message->data, min_range_m, max_range_m);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_lidar_range_m_ = message->data;
      profiles_[active_mode_].obstacle_influence_radius_m = message->data;
      profiles_[active_mode_].fw_avoid_trigger_distance_m =
        std::max(1.0, message->data - 10.0);
      applyActiveParameters();
    }
    RCLCPP_INFO(
      get_logger(), "APF influence range set to %.1f m (FW trigger %.1f m)",
      active_lidar_range_m_, active_lidar_range_m_ - 10.0);
  }

  ApfParameters readProfileParameters(
    const std::string & mode, const ApfParameters & fallback)
  {
    constexpr double radians_to_degrees = 57.29577951308232;
    constexpr double degrees_to_radians = 0.017453292519943295;
    const std::string prefix = "profiles." + mode + ".";
    ApfParameters parameters;
    parameters.obstacle_influence_radius_m = declare_parameter<double>(
      prefix + "obstacle_influence_radius", fallback.obstacle_influence_radius_m);
    parameters.fw_avoid_trigger_distance_m = declare_parameter<double>(
      prefix + "fw_avoid_trigger_dist", fallback.fw_avoid_trigger_distance_m);
    parameters.mc_corridor_half_width_m = declare_parameter<double>(
      prefix + "mc_trail_half_width", fallback.mc_corridor_half_width_m);
    parameters.fw_corridor_half_width_m = declare_parameter<double>(
      prefix + "fw_trail_half_width", fallback.fw_corridor_half_width_m);
    parameters.lidar_vertical_half_fov_rad = declare_parameter<double>(
      prefix + "lidar_vertical_half_fov_deg",
      fallback.lidar_vertical_half_fov_rad * radians_to_degrees) * degrees_to_radians;
    parameters.mc_attractive_gain = declare_parameter<double>(
      prefix + "mc_attractive_gain", fallback.mc_attractive_gain);
    parameters.fw_attractive_gain = declare_parameter<double>(
      prefix + "fw_attractive_gain", fallback.fw_attractive_gain);
    parameters.mc_repulsive_gain = declare_parameter<double>(
      prefix + "mc_repulsive_gain", fallback.mc_repulsive_gain);
    parameters.fw_repulsive_gain = declare_parameter<double>(
      prefix + "fw_repulsive_gain", fallback.fw_repulsive_gain);
    parameters.repulsive_distance_power = declare_parameter<double>(
      prefix + "repulsive_distance_power", fallback.repulsive_distance_power);
    parameters.fw_max_avoid_yaw_rad = declare_parameter<double>(
      prefix + "fw_max_avoid_angle_deg",
      fallback.fw_max_avoid_yaw_rad * radians_to_degrees) * degrees_to_radians;
    parameters.fw_max_avoid_pitch_rad = declare_parameter<double>(
      prefix + "fw_max_avoid_pitch_deg",
      fallback.fw_max_avoid_pitch_rad * radians_to_degrees) * degrees_to_radians;
    parameters.vertical_escape_pitch_gain = declare_parameter<double>(
      prefix + "vertical_escape_pitch_gain", fallback.vertical_escape_pitch_gain);
    parameters.clearance_radius_m = declare_parameter<double>(
      prefix + "apf_clearance_radius", fallback.clearance_radius_m);
    parameters.mc_max_horizontal_speed_m_s = declare_parameter<double>(
      prefix + "mc_speed", fallback.mc_max_horizontal_speed_m_s);
    parameters.mc_max_climb_speed_m_s = declare_parameter<double>(
      prefix + "mc_climb_speed", fallback.mc_max_climb_speed_m_s);
    parameters.clear_hold_time_s = declare_parameter<double>(
      prefix + "avoidance_clear_hold_time", fallback.clear_hold_time_s);
    parameters.sector_margin_min_rad = fallback.sector_margin_min_rad;
    parameters.sector_margin_max_rad = fallback.sector_margin_max_rad;
    parameters.sector_margin_speed_min_m_s = fallback.sector_margin_speed_min_m_s;
    parameters.sector_margin_speed_max_m_s = fallback.sector_margin_speed_max_m_s;
    parameters.direction_min_speed_m_s = fallback.direction_min_speed_m_s;
    parameters.emergency_radius_m = fallback.emergency_radius_m;
    return parameters;
  }

  sensor_msgs::msg::PointCloud2 makeDebugCloud(
    const std_msgs::msg::Header & header, const std::vector<Vec3> & relative_points,
    const VehicleState & state) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(relative_points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : relative_points) {
      *x = static_cast<float>(point.x + state.position_enu.x);
      *y = static_cast<float>(point.y + state.position_enu.y);
      *z = static_cast<float>(point.z + state.position_enu.z);
      ++x;
      ++y;
      ++z;
    }
    return cloud;
  }

  void publishModeTelemetry()
  {
    if (!telemetry_pub_) return;
    ApfTelemetry telemetry;
    telemetry.header.stamp = now();
    telemetry.header.frame_id = "map";
    telemetry.active_mode = active_mode_;
    telemetry.nearest_path_obstacle_distance_m = -1.0;
    telemetry_pub_->publish(telemetry);
  }

  void onCommand(const MotionCommand::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = *message;
    command_received_ = now();
    have_command_ = true;
  }

  void onState(const VehicleState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = *message;
    have_state_ = true;
  }

  void onObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    std::vector<Vec3> points;
    points.reserve(cloud->width * cloud->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) points.push_back({*x, *y, *z});
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Invalid obstacle cloud: %s", error.what());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_points_map_ = std::move(points);
    obstacles_received_ = now();
    have_obstacles_ = true;
  }

  void onTimer()
  {
    MotionCommand command;
    VehicleState state;
    std::vector<Vec3> map_points;
    rclcpp::Time command_received;
    rclcpp::Time obstacles_received;
    bool have_command = false;
    bool have_state = false;
    bool have_obstacles = false;
    ApfParameters active_parameters;
    double obstacle_timeout_s = 0.5;
    double debug_cloud_publish_period_s = 0.2;
    bool avoidance_enabled = true;
    std::string active_mode;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command = command_;
      state = state_;
      map_points = obstacle_points_map_;
      command_received = command_received_;
      obstacles_received = obstacles_received_;
      have_command = have_command_;
      have_state = have_state_;
      have_obstacles = have_obstacles_;
      active_parameters = active_parameters_;
      obstacle_timeout_s = obstacle_timeout_s_;
      debug_cloud_publish_period_s = debug_cloud_publish_period_s_;
      avoidance_enabled = avoidance_enabled_;
      active_mode = active_mode_;
    }

    if (!have_command || !have_state || !command.active ||
      (now() - command_received).seconds() > command_timeout_s_)
    {
      MotionCommand inactive;
      inactive.header.stamp = now();
      inactive.header.frame_id = "map";
      safe_pub_->publish(inactive);
      std::lock_guard<std::mutex> lock(mutex_);
      solver_.reset();
      return;
    }

    std::vector<Vec3> relative_points;
    if (have_obstacles && (now() - obstacles_received).seconds() <= obstacle_timeout_s) {
      relative_points.reserve(map_points.size());
      for (const auto & point : map_points) {
        relative_points.push_back({
          point.x - state.position_enu.x,
          point.y - state.position_enu.y,
          point.z - state.position_enu.z});
      }
    } else if (avoidance_enabled) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "APF has no fresh obstacle cloud");
    }

    const Vec3 desired{
      command.velocity_enu.x, command.velocity_enu.y, command.velocity_enu.z};
    const Vec3 current_velocity{
      state.velocity_enu.x, state.velocity_enu.y, state.velocity_enu.z};
    constexpr double half_pi = 1.5707963267948966;
    const double heading_enu_rad = state.attitude_valid &&
      std::isfinite(state.heading_ned_rad) ?
      half_pi - state.heading_ned_rad : std::numeric_limits<double>::quiet_NaN();
    drone_navigation::ApfResult result;
    if (avoidance_enabled && !relative_points.empty()) {
      const auto mode = command.vehicle_mode == MotionCommand::MODE_FIXED_WING ?
        FlightMode::FIXED_WING : FlightMode::MULTICOPTER;
      std::lock_guard<std::mutex> lock(mutex_);
      result = solver_.update(
          desired, current_velocity, heading_enu_rad, relative_points,
          mode, now().seconds());
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      solver_.reset();
      result.active_sector = calculateActiveSector(
        current_velocity, desired, heading_enu_rad, active_parameters);
      result.safe_velocity = desired;
      const double speed = std::sqrt(
        desired.x * desired.x + desired.y * desired.y + desired.z * desired.z);
      if (speed > 0.05) {
        result.attractive = {desired.x / speed, desired.y / speed, desired.z / speed};
      }
    }

    if ((now() - last_debug_cloud_publish_).seconds() >= debug_cloud_publish_period_s) {
      const auto debug_header = command.header;
      used_obstacles_pub_->publish(makeDebugCloud(
        debug_header, result.used_obstacles, state));
      ignored_obstacles_pub_->publish(makeDebugCloud(
        debug_header, result.sector_ignored_obstacles, state));
      last_debug_cloud_publish_ = now();
    }

    command.header.stamp = now();
    command.velocity_enu.x = result.safe_velocity.x;
    command.velocity_enu.y = result.safe_velocity.y;
    command.velocity_enu.z = result.safe_velocity.z;
    safe_pub_->publish(command);

    ApfTelemetry telemetry;
    telemetry.header = command.header;
    telemetry.vehicle_mode = command.vehicle_mode;
    telemetry.command_source = command.source;
    telemetry.avoidance_active = result.avoidance_active;
    telemetry.active_mode = active_mode;
    telemetry.nearest_path_obstacle_distance_m =
      std::isfinite(result.nearest_path_obstacle_distance_m) ?
      result.nearest_path_obstacle_distance_m : -1.0;
    telemetry.attractive_force_enu.x = result.attractive.x;
    telemetry.attractive_force_enu.y = result.attractive.y;
    telemetry.attractive_force_enu.z = result.attractive.z;
    telemetry.repulsive_force_enu.x = result.repulsive.x;
    telemetry.repulsive_force_enu.y = result.repulsive.y;
    telemetry.repulsive_force_enu.z = result.repulsive.z;
    telemetry.safe_command_enu = command.velocity_enu;
    telemetry.current_motion_direction_rad =
      result.active_sector.current_direction_rad;
    telemetry.desired_motion_direction_rad =
      result.active_sector.desired_direction_rad;
    telemetry.sector_center_rad = result.active_sector.center_rad;
    telemetry.sector_half_width_rad = result.active_sector.half_width_rad;
    telemetry.sector_margin_rad = result.active_sector.margin_rad;
    telemetry.emergency_radius_m = std::min(
      active_parameters.emergency_radius_m,
      active_parameters.obstacle_influence_radius_m);
    telemetry.current_direction_uses_fallback =
      result.active_sector.current_uses_fallback;
    telemetry.desired_direction_uses_fallback =
      result.active_sector.desired_uses_fallback;
    telemetry.used_obstacle_count = result.used_obstacles.size();
    telemetry.sector_ignored_obstacle_count = result.sector_ignored_obstacles.size();
    telemetry_pub_->publish(telemetry);
  }

  ApfParameters base_parameters_;
  ApfParameters active_parameters_;
  ApfSolver solver_;
  std::map<std::string, ApfParameters> profiles_;
  std::string active_mode_ {"normal"};
  double command_timeout_s_ {0.5};
  double obstacle_timeout_s_ {0.5};
  double active_lidar_range_m_ {70.0};
  double debug_cloud_publish_period_s_ {0.2};
  bool avoidance_enabled_ {true};
  std::mutex mutex_;
  MotionCommand command_;
  VehicleState state_;
  std::vector<Vec3> obstacle_points_map_;
  rclcpp::Time command_received_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacles_received_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_cloud_publish_ {0, 0, RCL_ROS_TIME};
  bool have_command_ {false};
  bool have_state_ {false};
  bool have_obstacles_ {false};
  rclcpp::Subscription<MotionCommand>::SharedPtr selected_sub_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lidar_range_sub_;
  rclcpp::Service<SetBool>::SharedPtr enabled_service_;
  rclcpp::Service<SetApfMode>::SharedPtr mode_service_;
  rclcpp::Publisher<MotionCommand>::SharedPtr safe_pub_;
  rclcpp::Publisher<ApfTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr used_obstacles_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ignored_obstacles_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApfSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
