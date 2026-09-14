#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

using MotionCommand = drone_interfaces::msg::MotionCommand;
using VehicleState = drone_interfaces::msg::VehicleState;
using VC = px4_msgs::msg::VehicleCommand;
using VS = px4_msgs::msg::VehicleStatus;
using std::placeholders::_1;

class Px4GatewayNode : public rclcpp::Node
{
public:
  Px4GatewayNode()
  : Node("px4_gateway")
  {
    command_timeout_s_ = std::max(
      0.1, declare_parameter<double>("command_timeout_s", 0.5));
    land_detected_timeout_s_ = std::max(
      0.2, declare_parameter<double>("land_detected_timeout_s", 1.0));
    fw_lookahead_m_ = std::max(
      5.0, declare_parameter<double>("fw_lookahead_m", 40.0));
    max_horizontal_speed_m_s_ = std::max(
      1.0, declare_parameter<double>("max_horizontal_speed_m_s", 25.0));
    max_vertical_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("max_vertical_speed_m_s", 5.0));
    minimum_altitude_m_ = std::max(
      0.0, declare_parameter<double>("minimum_altitude_m", 0.5));
    target_system_ = static_cast<uint8_t>(std::clamp(
      declare_parameter<int64_t>("target_system", 1), int64_t{1}, int64_t{255}));
    map_origin_east_m_ = declare_parameter<double>("map_origin_east_m", 0.0);
    map_origin_north_m_ = declare_parameter<double>("map_origin_north_m", 0.0);
    map_origin_up_m_ = declare_parameter<double>("map_origin_up_m", 0.0);

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    status_sub_ = create_subscription<VS>(
      "/fmu/out/vehicle_status_v4", px4_qos,
      std::bind(&Px4GatewayNode::onStatus, this, _1));
    land_detected_sub_ =
      create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      std::bind(&Px4GatewayNode::onLandDetected, this, _1));
    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos,
      std::bind(&Px4GatewayNode::onPosition, this, _1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", px4_qos,
      std::bind(&Px4GatewayNode::onAttitude, this, _1));
    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status_v1", px4_qos,
      std::bind(&Px4GatewayNode::onBattery, this, _1));
    safe_command_sub_ = create_subscription<MotionCommand>(
      "/motion/safe_command", 10,
      std::bind(&Px4GatewayNode::onSafeCommand, this, _1));
    vehicle_command_sub_ = create_subscription<VC>(
      "/px4/vehicle_command_request", 10,
      std::bind(&Px4GatewayNode::onVehicleCommand, this, _1));

    state_pub_ = create_publisher<VehicleState>("/vehicle/state", 10);
    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<VC>("/fmu/in/vehicle_command", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&Px4GatewayNode::onTimer, this));
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&Px4GatewayNode::onParametersSet, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "PX4 gateway ready; sole owner of /fmu/in setpoints and commands");
  }

private:
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    try {
      std::lock_guard<std::mutex> lock(config_mutex_);
      double fw_lookahead = fw_lookahead_m_;
      double max_horizontal_speed = max_horizontal_speed_m_s_;
      double max_vertical_speed = max_vertical_speed_m_s_;
      double minimum_altitude = minimum_altitude_m_;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "fw_lookahead_m") fw_lookahead = parameter.as_double();
        else if (parameter.get_name() == "max_horizontal_speed_m_s") {
          max_horizontal_speed = parameter.as_double();
        } else if (parameter.get_name() == "max_vertical_speed_m_s") {
          max_vertical_speed = parameter.as_double();
        } else if (parameter.get_name() == "minimum_altitude_m") {
          minimum_altitude = parameter.as_double();
        }
      }
      if (!std::isfinite(fw_lookahead) || fw_lookahead < 5.0 || fw_lookahead > 300.0) {
        result.successful = false;
        result.reason = "fixed-wing lookahead must be between 5 and 300 m";
      } else if (!std::isfinite(max_horizontal_speed) || max_horizontal_speed < 1.0 ||
        max_horizontal_speed > 100.0)
      {
        result.successful = false;
        result.reason = "horizontal speed limit must be between 1 and 100 m/s";
      } else if (!std::isfinite(max_vertical_speed) || max_vertical_speed < 0.2 ||
        max_vertical_speed > 30.0)
      {
        result.successful = false;
        result.reason = "vertical speed limit must be between 0.2 and 30 m/s";
      } else if (!std::isfinite(minimum_altitude) || minimum_altitude < 0.0 ||
        minimum_altitude > 100.0)
      {
        result.successful = false;
        result.reason = "minimum altitude must be between 0 and 100 m";
      }
      if (result.successful) {
        fw_lookahead_m_ = fw_lookahead;
        max_horizontal_speed_m_s_ = max_horizontal_speed;
        max_vertical_speed_m_s_ = max_vertical_speed;
        minimum_altitude_m_ = minimum_altitude;
      }
    } catch (const std::exception & exception) {
      result.successful = false;
      result.reason = exception.what();
    }
    if (result.successful) {
      RCLCPP_INFO(get_logger(), "Runtime PX4 parameter update accepted");
    }
    return result;
  }

  static float nan()
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  uint64_t timestampUs() const
  {
    return static_cast<uint64_t>(now().nanoseconds() / 1000ULL);
  }

  void onStatus(const VS::SharedPtr message)
  {
    status_ = *message;
    have_status_ = true;
  }

  void onPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    position_ = *message;
    have_position_ = true;
  }

  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr message)
  {
    land_detected_ = *message;
    land_detected_received_ = now();
    have_land_detected_ = true;
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr message)
  {
    attitude_ = *message;
    have_attitude_ = true;
  }

  void onBattery(const px4_msgs::msg::BatteryStatus::SharedPtr message)
  {
    battery_ = *message;
    have_battery_ = true;
  }

  void onSafeCommand(const MotionCommand::SharedPtr message)
  {
    command_ = *message;
    command_received_ = now();
    have_command_ = true;
    if (message->active) last_active_command_ = *message;
  }

  void onVehicleCommand(const VC::SharedPtr request)
  {
    auto command = *request;
    command.timestamp = timestampUs();
    command.target_system = target_system_;
    command.target_component = 1;
    command.source_system = 1;
    command.source_component = 1;
    command.from_external = true;
    vehicle_command_pub_->publish(command);
  }

  uint8_t currentMode() const
  {
    if (!have_status_) return VehicleState::MODE_UNKNOWN;
    return status_.vehicle_type == VS::VEHICLE_TYPE_FIXED_WING ?
      VehicleState::MODE_FIXED_WING : VehicleState::MODE_MULTICOPTER;
  }

  void publishState()
  {
    VehicleState state;
    state.header.stamp = now();
    state.header.frame_id = "map";
    state.position_valid = have_position_ && position_.xy_valid && position_.z_valid;
    state.attitude_valid = have_attitude_;
    state.armed = have_status_ && status_.arming_state == VS::ARMING_STATE_ARMED;
    state.offboard = have_status_ && status_.nav_state == VS::NAVIGATION_STATE_OFFBOARD;
    state.landed_valid = have_land_detected_ &&
      (now() - land_detected_received_).seconds() <= land_detected_timeout_s_;
    state.landed = state.landed_valid && land_detected_.landed;
    state.vehicle_mode = currentMode();
    state.battery_warning = VehicleState::BATTERY_WARNING_UNKNOWN;
    if (have_position_) {
      state.position_enu.x = map_origin_east_m_ + position_.y;
      state.position_enu.y = map_origin_north_m_ + position_.x;
      state.position_enu.z = map_origin_up_m_ - position_.z;
      state.velocity_enu.x = position_.vy;
      state.velocity_enu.y = position_.vx;
      state.velocity_enu.z = -position_.vz;
      state.heading_ned_rad = position_.heading;
    }
    if (have_attitude_) {
      state.attitude_body_to_ned.w = attitude_.q[0];
      state.attitude_body_to_ned.x = attitude_.q[1];
      state.attitude_body_to_ned.y = attitude_.q[2];
      state.attitude_body_to_ned.z = attitude_.q[3];
    }
    if (have_battery_) {
      state.battery_connected = battery_.connected;
      state.battery_valid = battery_.connected && std::isfinite(battery_.remaining) &&
        battery_.remaining >= 0.0F && battery_.remaining <= 1.0F;
      if (state.battery_valid) {
        state.battery_remaining_pct = 100.0 * battery_.remaining;
      }
      state.battery_time_remaining_s = std::isfinite(battery_.time_remaining_s) ?
        battery_.time_remaining_s : -1.0;
      state.battery_voltage_v = battery_.voltage_v;
      state.battery_current_a = battery_.current_a;
      state.battery_power_w = battery_.voltage_v * std::max(battery_.current_a, 0.0F);
      const double nominal_voltage_v = 3.7 * battery_.cell_count;
      state.battery_capacity_wh = nominal_voltage_v * battery_.capacity / 1000.0;
      state.battery_remaining_energy_wh = state.battery_valid ?
        state.battery_capacity_wh * battery_.remaining : 0.0;
      state.battery_warning = battery_.warning <= px4_msgs::msg::BatteryStatus::WARNING_FAILED ?
        battery_.warning : VehicleState::BATTERY_WARNING_UNKNOWN;
    }
    state_pub_->publish(state);
  }

  MotionCommand activeCommand() const
  {
    if (have_command_ && command_.active &&
      (now() - command_received_).seconds() <= command_timeout_s_)
    {
      return command_;
    }
    if (currentMode() == VehicleState::MODE_FIXED_WING && last_active_command_.active) {
      return last_active_command_;
    }
    MotionCommand stopped;
    stopped.vehicle_mode = MotionCommand::MODE_MULTICOPTER;
    return stopped;
  }

  void publishSetpoint(const MotionCommand & command)
  {
    double fw_lookahead_m = 40.0;
    double max_horizontal_speed_m_s = 25.0;
    double max_vertical_speed_m_s = 5.0;
    double minimum_altitude_m = 0.5;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      fw_lookahead_m = fw_lookahead_m_;
      max_horizontal_speed_m_s = max_horizontal_speed_m_s_;
      max_vertical_speed_m_s = max_vertical_speed_m_s_;
      minimum_altitude_m = minimum_altitude_m_;
    }
    double east_velocity = command.velocity_enu.x;
    double north_velocity = command.velocity_enu.y;
    double up_velocity = command.velocity_enu.z;
    const double horizontal_speed = std::hypot(east_velocity, north_velocity);
    if (horizontal_speed > max_horizontal_speed_m_s) {
      const double scale = max_horizontal_speed_m_s / horizontal_speed;
      east_velocity *= scale;
      north_velocity *= scale;
    }
    up_velocity = std::clamp(
      up_velocity, -max_vertical_speed_m_s, max_vertical_speed_m_s);

    const bool fixed_wing = command.vehicle_mode == MotionCommand::MODE_FIXED_WING;
    const bool use_fw_position = fixed_wing && command.hold_altitude && have_position_ &&
      std::hypot(east_velocity, north_velocity) > 0.1;
    const bool use_mc_altitude_hold = !fixed_wing && command.hold_altitude &&
      have_position_ && std::fabs(up_velocity) < 0.05;
    px4_msgs::msg::OffboardControlMode heartbeat;
    heartbeat.timestamp = timestampUs();
    heartbeat.position = use_fw_position || use_mc_altitude_hold;
    heartbeat.velocity = true;
    offboard_pub_->publish(heartbeat);

    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = timestampUs();
    const double target_altitude_local_m = command.target_altitude_m - map_origin_up_m_;
    if (use_fw_position) {
      const double speed = std::hypot(east_velocity, north_velocity);
      const double current_altitude = -position_.z;
      const double altitude_base = up_velocity > 0.0 ?
        std::max(target_altitude_local_m, current_altitude) :
        target_altitude_local_m;
      const double projected_altitude = std::max(
        minimum_altitude_m,
        altitude_base + fw_lookahead_m * up_velocity / speed);
      setpoint.position = {
        static_cast<float>(position_.x + fw_lookahead_m * north_velocity / speed),
        static_cast<float>(position_.y + fw_lookahead_m * east_velocity / speed),
        static_cast<float>(-projected_altitude)};
    } else if (use_mc_altitude_hold) {
      setpoint.position = {
        nan(), nan(),
        static_cast<float>(-std::max(minimum_altitude_m, target_altitude_local_m))};
    } else {
      setpoint.position = {nan(), nan(), nan()};
    }
    setpoint.velocity = {
      static_cast<float>(north_velocity),
      static_cast<float>(east_velocity),
      static_cast<float>(-up_velocity)};
    setpoint.acceleration = {nan(), nan(), nan()};
    setpoint.jerk = {nan(), nan(), nan()};
    setpoint.yaw = nan();
    setpoint.yawspeed = fixed_wing ? nan() : static_cast<float>(command.yaw_rate_ned);
    setpoint_pub_->publish(setpoint);
  }

  void onTimer()
  {
    publishState();
    publishSetpoint(activeCommand());
  }

  double command_timeout_s_ {0.5};
  double land_detected_timeout_s_ {1.0};
  double fw_lookahead_m_ {40.0};
  double max_horizontal_speed_m_s_ {25.0};
  double max_vertical_speed_m_s_ {5.0};
  double minimum_altitude_m_ {0.5};
  double map_origin_east_m_ {0.0};
  double map_origin_north_m_ {0.0};
  double map_origin_up_m_ {0.0};
  std::mutex config_mutex_;
  uint8_t target_system_ {1};
  VS status_;
  px4_msgs::msg::VehicleLandDetected land_detected_;
  px4_msgs::msg::VehicleLocalPosition position_;
  px4_msgs::msg::VehicleAttitude attitude_;
  px4_msgs::msg::BatteryStatus battery_;
  MotionCommand command_;
  MotionCommand last_active_command_;
  bool have_status_ {false};
  bool have_land_detected_ {false};
  bool have_position_ {false};
  bool have_attitude_ {false};
  bool have_battery_ {false};
  bool have_command_ {false};
  rclcpp::Time command_received_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time land_detected_received_ {0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<VS>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<MotionCommand>::SharedPtr safe_command_sub_;
  rclcpp::Subscription<VC>::SharedPtr vehicle_command_sub_;
  rclcpp::Publisher<VehicleState>::SharedPtr state_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VC>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4GatewayNode>());
  rclcpp::shutdown();
  return 0;
}
