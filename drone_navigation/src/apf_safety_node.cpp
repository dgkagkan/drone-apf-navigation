#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <drone_interfaces/msg/apf_telemetry.hpp>
#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "drone_navigation/apf_solver.hpp"

using ApfTelemetry = drone_interfaces::msg::ApfTelemetry;
using MotionCommand = drone_interfaces::msg::MotionCommand;
using VehicleState = drone_interfaces::msg::VehicleState;
using drone_navigation::ApfParameters;
using drone_navigation::ApfSolver;
using drone_navigation::FlightMode;
using drone_navigation::Vec3;
using std::placeholders::_1;

class ApfSafetyNode : public rclcpp::Node
{
public:
  ApfSafetyNode()
  : Node("apf_safety"), solver_(readParameters())
  {
    command_timeout_s_ = std::max(
      0.1, declare_parameter<double>("command_timeout_s", 0.5));
    obstacle_timeout_s_ = std::max(
      0.1, declare_parameter<double>("obstacle_timeout_s", 0.5));
    avoidance_enabled_ = declare_parameter<bool>("avoidance_enabled", true);

    selected_sub_ = create_subscription<MotionCommand>(
      "/motion/selected_intent", 10, std::bind(&ApfSafetyNode::onCommand, this, _1));
    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&ApfSafetyNode::onState, this, _1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/perception/obstacles", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ApfSafetyNode::onObstacles, this, _1));
    safe_pub_ = create_publisher<MotionCommand>("/motion/safe_command", 10);
    telemetry_pub_ = create_publisher<ApfTelemetry>("/apf/telemetry", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&ApfSafetyNode::onTimer, this));
  }

private:
  ApfParameters readParameters()
  {
    constexpr double degrees_to_radians = 0.017453292519943295;
    ApfParameters parameters;
    parameters.obstacle_influence_radius_m =
      declare_parameter<double>("obstacle_influence_radius", 77.96063615194443);
    parameters.fw_avoid_trigger_distance_m =
      declare_parameter<double>("fw_avoid_trigger_dist", 73.5424773407324);
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
    return parameters;
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
    }

    if (!have_command || !have_state || !command.active ||
      (now() - command_received).seconds() > command_timeout_s_)
    {
      MotionCommand inactive;
      inactive.header.stamp = now();
      inactive.header.frame_id = "map";
      safe_pub_->publish(inactive);
      solver_.reset();
      return;
    }

    std::vector<Vec3> relative_points;
    if (have_obstacles && (now() - obstacles_received).seconds() <= obstacle_timeout_s_) {
      relative_points.reserve(map_points.size());
      for (const auto & point : map_points) {
        relative_points.push_back({
          point.x - state.position_enu.x,
          point.y - state.position_enu.y,
          point.z - state.position_enu.z});
      }
    } else if (avoidance_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "APF has no fresh obstacle cloud");
    }

    const Vec3 desired{
      command.velocity_enu.x, command.velocity_enu.y, command.velocity_enu.z};
    drone_navigation::ApfResult result;
    if (avoidance_enabled_ && !relative_points.empty()) {
      const auto mode = command.vehicle_mode == MotionCommand::MODE_FIXED_WING ?
        FlightMode::FIXED_WING : FlightMode::MULTICOPTER;
      result = solver_.update(desired, relative_points, mode, now().seconds());
    } else {
      solver_.reset();
      result.safe_velocity = desired;
      const double speed = std::sqrt(
        desired.x * desired.x + desired.y * desired.y + desired.z * desired.z);
      if (speed > 0.05) {
        result.attractive = {desired.x / speed, desired.y / speed, desired.z / speed};
      }
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
    telemetry_pub_->publish(telemetry);
  }

  ApfSolver solver_;
  double command_timeout_s_ {0.5};
  double obstacle_timeout_s_ {0.5};
  bool avoidance_enabled_ {true};
  std::mutex mutex_;
  MotionCommand command_;
  VehicleState state_;
  std::vector<Vec3> obstacle_points_map_;
  rclcpp::Time command_received_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacles_received_ {0, 0, RCL_ROS_TIME};
  bool have_command_ {false};
  bool have_state_ {false};
  bool have_obstacles_ {false};
  rclcpp::Subscription<MotionCommand>::SharedPtr selected_sub_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr safe_pub_;
  rclcpp::Publisher<ApfTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApfSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
