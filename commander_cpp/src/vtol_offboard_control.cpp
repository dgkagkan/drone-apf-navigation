// vtol_offboard_control
// ---------------------
// PX4-facing offboard bridge. It accepts ROS ENU velocity commands on
// /apf/velocity_setpoint, converts them to PX4 NED TrajectorySetpoint messages,
// and continuously streams OffboardControlMode heartbeats.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

class VtolOffboardControl : public rclcpp::Node
{
public:
  VtolOffboardControl()
  : Node("vtol_offboard_control")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/apf/velocity_setpoint");
    engage_ = declare_parameter<bool>("engage", false);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    max_speed_ = declare_parameter<double>("max_speed", 4.0);
    max_climb_rate_ = declare_parameter<double>("max_climb_rate", 1.5);
    command_warmup_ticks_ = declare_parameter<int>("command_warmup_ticks", 10);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);

    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();

    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4_qos,
      std::bind(&VtolOffboardControl::onStatus, this, _1));

    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic_, 10, std::bind(&VtolOffboardControl::onVelocitySetpoint, this, _1));

    engage_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/apf/engage", 10, std::bind(&VtolOffboardControl::onEngage, this, _1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&VtolOffboardControl::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "vtol_offboard_control: listening on %s, engage=%s auto_arm=%s",
      input_topic_.c_str(), engage_ ? "true" : "false", auto_arm_ ? "true" : "false");
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

  void onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    status_ = *msg;
    have_status_ = true;
  }

  void onVelocitySetpoint(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    latest_velocity_ = *msg;
    latest_velocity_time_ = now();
    have_velocity_ = true;
  }

  void onEngage(const std_msgs::msg::Bool::SharedPtr msg)
  {
    engage_ = msg->data;
    if (!engage_) {
      requested_offboard_ = false;
      requested_arm_ = false;
    }

    RCLCPP_INFO(get_logger(), "APF offboard engage=%s", engage_ ? "true" : "false");
  }

  void onTimer()
  {
    publishOffboardHeartbeat();
    publishTrajectorySetpoint();

    if (!engage_) {
      warmup_ticks_ = 0;
      return;
    }

    warmup_ticks_++;
    if (warmup_ticks_ < command_warmup_ticks_) {
      return;
    }

    if (!requested_offboard_ || !isOffboard()) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      requested_offboard_ = true;
      RCLCPP_INFO(get_logger(), "Requested PX4 offboard mode");
    }

    if (auto_arm_ && !requested_arm_ && !isArmed()) {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM));
      requested_arm_ = true;
      RCLCPP_INFO(get_logger(), "Requested arm");
    }
  }

  bool isOffboard() const
  {
    return have_status_ &&
           status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool isArmed() const
  {
    return have_status_ &&
           status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool velocityFresh() const
  {
    if (!have_velocity_) {
      return false;
    }

    return (now() - latest_velocity_time_).seconds() <= command_timeout_s_;
  }

  void publishOffboardHeartbeat()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestampUs();
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_pub_->publish(msg);
  }

  void publishTrajectorySetpoint()
  {
    double vx_enu = 0.0;
    double vy_enu = 0.0;
    double vz_enu = 0.0;

    if (engage_ && velocityFresh()) {
      vx_enu = latest_velocity_.twist.linear.x;
      vy_enu = latest_velocity_.twist.linear.y;
      vz_enu = latest_velocity_.twist.linear.z;
    }

    const double horizontal = std::hypot(vx_enu, vy_enu);
    if (horizontal > max_speed_ && horizontal > 1e-6) {
      const double scale = max_speed_ / horizontal;
      vx_enu *= scale;
      vy_enu *= scale;
    }
    vz_enu = std::clamp(vz_enu, -max_climb_rate_, max_climb_rate_);

    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = timestampUs();
    msg.position = {nan(), nan(), nan()};
    msg.velocity = {
      static_cast<float>(vy_enu),   // ENU y/North -> NED x
      static_cast<float>(vx_enu),   // ENU x/East  -> NED y
      static_cast<float>(-vz_enu)}; // ENU z/Up    -> NED z/Down
    msg.acceleration = {nan(), nan(), nan()};
    msg.jerk = {nan(), nan(), nan()};

    if (horizontal > 0.1) {
      const double yaw_enu = std::atan2(vy_enu, vx_enu);
      msg.yaw = static_cast<float>(M_PI_2 - yaw_enu);
    } else {
      msg.yaw = nan();
    }
    msg.yawspeed = nan();

    setpoint_pub_->publish(msg);
  }

  void publishVehicleCommand(
    uint32_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
    float param4 = 0.0f, double param5 = 0.0, double param6 = 0.0, float param7 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestampUs();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  std::string input_topic_;
  bool engage_ {false};
  bool auto_arm_ {false};
  bool requested_offboard_ {false};
  bool requested_arm_ {false};
  bool have_status_ {false};
  bool have_velocity_ {false};
  int warmup_ticks_ {0};
  int command_warmup_ticks_ {10};
  double max_speed_ {4.0};
  double max_climb_rate_ {1.5};
  double command_timeout_s_ {0.5};

  px4_msgs::msg::VehicleStatus status_;
  geometry_msgs::msg::TwistStamped latest_velocity_;
  rclcpp::Time latest_velocity_time_ {0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VtolOffboardControl>());
  rclcpp::shutdown();
  return 0;
}
