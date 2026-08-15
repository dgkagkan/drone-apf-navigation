#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
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
    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos,
      std::bind(&Px4GatewayNode::onPosition, this, _1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", px4_qos,
      std::bind(&Px4GatewayNode::onAttitude, this, _1));
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

    RCLCPP_INFO(
      get_logger(), "PX4 gateway ready; sole owner of /fmu/in setpoints and commands");
  }

private:
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

  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr message)
  {
    attitude_ = *message;
    have_attitude_ = true;
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
    state.vehicle_mode = currentMode();
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
    double east_velocity = command.velocity_enu.x;
    double north_velocity = command.velocity_enu.y;
    double up_velocity = command.velocity_enu.z;
    const double horizontal_speed = std::hypot(east_velocity, north_velocity);
    if (horizontal_speed > max_horizontal_speed_m_s_) {
      const double scale = max_horizontal_speed_m_s_ / horizontal_speed;
      east_velocity *= scale;
      north_velocity *= scale;
    }
    up_velocity = std::clamp(
      up_velocity, -max_vertical_speed_m_s_, max_vertical_speed_m_s_);

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
        minimum_altitude_m_,
        altitude_base + fw_lookahead_m_ * up_velocity / speed);
      setpoint.position = {
        static_cast<float>(position_.x + fw_lookahead_m_ * north_velocity / speed),
        static_cast<float>(position_.y + fw_lookahead_m_ * east_velocity / speed),
        static_cast<float>(-projected_altitude)};
    } else if (use_mc_altitude_hold) {
      setpoint.position = {
        nan(), nan(),
        static_cast<float>(-std::max(minimum_altitude_m_, target_altitude_local_m))};
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
  double fw_lookahead_m_ {40.0};
  double max_horizontal_speed_m_s_ {25.0};
  double max_vertical_speed_m_s_ {5.0};
  double minimum_altitude_m_ {0.5};
  double map_origin_east_m_ {0.0};
  double map_origin_north_m_ {0.0};
  double map_origin_up_m_ {0.0};
  uint8_t target_system_ {1};
  VS status_;
  px4_msgs::msg::VehicleLocalPosition position_;
  px4_msgs::msg::VehicleAttitude attitude_;
  MotionCommand command_;
  MotionCommand last_active_command_;
  bool have_status_ {false};
  bool have_position_ {false};
  bool have_attitude_ {false};
  bool have_command_ {false};
  rclcpp::Time command_received_ {0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<VS>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<MotionCommand>::SharedPtr safe_command_sub_;
  rclcpp::Subscription<VC>::SharedPtr vehicle_command_sub_;
  rclcpp::Publisher<VehicleState>::SharedPtr state_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VC>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4GatewayNode>());
  rclcpp::shutdown();
  return 0;
}
