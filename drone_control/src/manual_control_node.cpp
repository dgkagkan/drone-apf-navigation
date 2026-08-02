#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include <drone_interfaces/msg/flight_request.hpp>
#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using FlightRequest = drone_interfaces::msg::FlightRequest;
using MotionCommand = drone_interfaces::msg::MotionCommand;
using VehicleState = drone_interfaces::msg::VehicleState;
using std::placeholders::_1;

class ManualControlNode : public rclcpp::Node
{
public:
  ManualControlNode()
  : Node("manual_control")
  {
    mc_speed_m_s_ = std::max(0.2, declare_parameter<double>("mc_speed", 4.0));
    mc_climb_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("mc_climb_speed", 2.0));
    mc_yaw_rate_rad_s_ = std::max(
      0.1, declare_parameter<double>("mc_yaw_rate_deg", 60.0)) * degrees_to_radians_;
    fw_speed_min_m_s_ = std::max(
      5.0, declare_parameter<double>("fw_speed_min", 10.0));
    fw_speed_max_m_s_ = std::max(
      fw_speed_min_m_s_, declare_parameter<double>("fw_speed_max", 20.0));
    fw_speed_cruise_m_s_ = std::clamp(
      declare_parameter<double>("fw_speed_cruise", 20.0),
      fw_speed_min_m_s_, fw_speed_max_m_s_);
    fw_turn_rate_rad_s_ = std::max(
      0.1, declare_parameter<double>("fw_turn_rate_deg", 25.0)) * degrees_to_radians_;
    fw_climb_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("fw_climb_speed", 4.0));
    minimum_altitude_m_ = std::max(
      0.0, declare_parameter<double>("minimum_altitude", 2.0));
    joy_timeout_s_ = std::max(
      0.1, declare_parameter<double>("joy_timeout", 0.5));

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ManualControlNode::onJoy, this, _1));
    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&ManualControlNode::onState, this, _1));
    intent_pub_ = create_publisher<MotionCommand>("/motion/manual_intent", 10);
    request_pub_ = create_publisher<FlightRequest>("/flight/request", 10);
    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&ManualControlNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Manual control ready: hold L1 for flight input, L1+L2 arm, "
      "L1+O disarm, O land, L1+TRIANGLE FW, L1+SQUARE MC");
  }

private:
  enum Axis : std::size_t
  {
    LEFT_X = 0,
    LEFT_Y = 1,
    RIGHT_X = 2,
    RIGHT_Y = 3,
    LEFT_TRIGGER = 4
  };

  enum Button : std::size_t
  {
    CROSS = 0,
    CIRCLE = 1,
    SQUARE = 2,
    TRIANGLE = 3,
    L1 = 9
  };

  double axis(std::size_t index) const
  {
    return index < axes_.size() ? std::clamp<double>(axes_[index], -1.0, 1.0) : 0.0;
  }

  bool button(std::size_t index) const
  {
    return index < buttons_.size() && buttons_[index] != 0;
  }

  bool previousButton(std::size_t index) const
  {
    return index < previous_buttons_.size() && previous_buttons_[index] != 0;
  }

  bool pressed(std::size_t index) const
  {
    return button(index) && !previousButton(index);
  }

  bool comboPressed(std::size_t first, std::size_t second) const
  {
    return button(first) && button(second) &&
      !(previousButton(first) && previousButton(second));
  }

  bool leftTriggerPressed() const
  {
    return left_trigger_initialized_ &&
      std::fabs(axis(LEFT_TRIGGER) - left_trigger_rest_) >= left_trigger_threshold_;
  }

  bool joyFresh() const
  {
    return have_joy_ && (now() - last_joy_).seconds() <= joy_timeout_s_;
  }

  void publishRequest(uint8_t request)
  {
    FlightRequest message;
    message.header.stamp = now();
    message.request = request;
    request_pub_->publish(message);
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr message)
  {
    const bool previous_left_trigger = leftTriggerPressed();
    previous_buttons_ = buttons_;
    axes_ = message->axes;
    buttons_ = message->buttons;
    last_joy_ = now();
    if (!left_trigger_initialized_ && LEFT_TRIGGER < axes_.size()) {
      left_trigger_rest_ = axis(LEFT_TRIGGER);
      left_trigger_initialized_ = true;
    }
    if (!have_joy_) {
      have_joy_ = true;
      RCLCPP_INFO(
        get_logger(), "Controller connected: %zu axes, %zu buttons",
        axes_.size(), buttons_.size());
    }

    const bool arm_combo = button(L1) && leftTriggerPressed() &&
      !(previousButton(L1) && previous_left_trigger);
    if (arm_combo) {
      publishRequest(FlightRequest::ARM_OFFBOARD);
    } else if (comboPressed(L1, CIRCLE)) {
      publishRequest(FlightRequest::DISARM);
    } else if (pressed(CIRCLE)) {
      publishRequest(FlightRequest::LAND);
    }

    if (comboPressed(L1, TRIANGLE)) {
      requested_mode_ = MotionCommand::MODE_FIXED_WING;
      initializeFwReferences();
      publishRequest(FlightRequest::TRANSITION_TO_FW);
    }
    if (comboPressed(L1, SQUARE)) {
      requested_mode_ = MotionCommand::MODE_MULTICOPTER;
      fw_references_initialized_ = false;
      publishRequest(FlightRequest::TRANSITION_TO_MC);
    }
  }

  void onState(const VehicleState::SharedPtr message)
  {
    state_ = *message;
    have_state_ = true;
    if (!target_altitude_initialized_ && message->position_valid) {
      target_altitude_m_ = std::max(minimum_altitude_m_, message->position_enu.z);
      target_altitude_initialized_ = true;
    }
    if (!requested_mode_initialized_) {
      requested_mode_ = message->vehicle_mode == VehicleState::MODE_FIXED_WING ?
        MotionCommand::MODE_FIXED_WING : MotionCommand::MODE_MULTICOPTER;
      requested_mode_initialized_ = true;
    }
  }

  void initializeFwReferences()
  {
    if (have_state_) {
      fw_heading_enu_rad_ = 1.5707963267948966 - state_.heading_ned_rad;
      target_altitude_m_ = std::max(minimum_altitude_m_, state_.position_enu.z);
      target_altitude_initialized_ = true;
    }
    fw_references_initialized_ = true;
  }

  void onTimer()
  {
    const auto stamp = now();
    const double dt = std::clamp((stamp - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = stamp;
    const bool fresh = joyFresh();
    const bool deadman = fresh && button(L1);

    MotionCommand command;
    command.header.stamp = stamp;
    command.header.frame_id = "map";
    command.source = MotionCommand::SOURCE_MANUAL;
    command.active = fresh;
    command.manual_override = deadman;
    command.vehicle_mode = requested_mode_;

    if (requested_mode_ == MotionCommand::MODE_FIXED_WING) {
      if (!fw_references_initialized_) initializeFwReferences();
      if (deadman) {
        const double turn_right = -axis(RIGHT_X);
        fw_heading_enu_rad_ -= turn_right * fw_turn_rate_rad_s_ * dt;
        target_altitude_m_ += -axis(RIGHT_Y) * fw_climb_speed_m_s_ * dt;
        target_altitude_m_ = std::max(minimum_altitude_m_, target_altitude_m_);
      }
      const double speed_input = deadman ? -axis(LEFT_Y) : 0.0;
      const double speed_span = speed_input >= 0.0 ?
        fw_speed_max_m_s_ - fw_speed_cruise_m_s_ :
        fw_speed_cruise_m_s_ - fw_speed_min_m_s_;
      const double speed = std::clamp(
        fw_speed_cruise_m_s_ + speed_input * speed_span,
        fw_speed_min_m_s_, fw_speed_max_m_s_);
      command.velocity_enu.x = speed * std::cos(fw_heading_enu_rad_);
      command.velocity_enu.y = speed * std::sin(fw_heading_enu_rad_);
      command.hold_altitude = true;
      command.target_altitude_m = target_altitude_m_;
    } else if (deadman && have_state_) {
      const double forward = -axis(LEFT_Y) * mc_speed_m_s_;
      const double left = -axis(LEFT_X) * mc_speed_m_s_;
      const double yaw_enu = 1.5707963267948966 - state_.heading_ned_rad;
      const double cosine = std::cos(yaw_enu);
      const double sine = std::sin(yaw_enu);
      command.velocity_enu.x = cosine * forward - sine * left;
      command.velocity_enu.y = sine * forward + cosine * left;
      command.velocity_enu.z = -axis(RIGHT_Y) * mc_climb_speed_m_s_;
      command.yaw_rate_ned = -axis(RIGHT_X) * mc_yaw_rate_rad_s_;
    }

    intent_pub_->publish(command);
  }

  static constexpr double degrees_to_radians_ = 0.017453292519943295;
  double mc_speed_m_s_ {4.0};
  double mc_climb_speed_m_s_ {2.0};
  double mc_yaw_rate_rad_s_ {1.0471975512};
  double fw_speed_min_m_s_ {10.0};
  double fw_speed_cruise_m_s_ {20.0};
  double fw_speed_max_m_s_ {20.0};
  double fw_turn_rate_rad_s_ {0.4363323130};
  double fw_climb_speed_m_s_ {4.0};
  double minimum_altitude_m_ {2.0};
  double joy_timeout_s_ {0.5};
  double left_trigger_rest_ {0.0};
  double left_trigger_threshold_ {0.5};
  std::vector<float> axes_;
  std::vector<int32_t> buttons_;
  std::vector<int32_t> previous_buttons_;
  VehicleState state_;
  bool have_joy_ {false};
  bool have_state_ {false};
  bool requested_mode_initialized_ {false};
  bool fw_references_initialized_ {false};
  bool target_altitude_initialized_ {false};
  bool left_trigger_initialized_ {false};
  uint8_t requested_mode_ {MotionCommand::MODE_MULTICOPTER};
  double fw_heading_enu_rad_ {0.0};
  double target_altitude_m_ {2.0};
  rclcpp::Time last_joy_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_ {0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr intent_pub_;
  rclcpp::Publisher<FlightRequest>::SharedPtr request_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualControlNode>());
  rclcpp::shutdown();
  return 0;
}
