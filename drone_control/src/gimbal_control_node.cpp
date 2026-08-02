#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>

using std::placeholders::_1;

class GimbalControlNode : public rclcpp::Node
{
public:
  GimbalControlNode()
  : Node("gimbal_control")
  {
    pan_rate_rad_s_ = std::max(
      0.05, declare_parameter<double>("pan_rate_deg_s", 60.0) * degrees_to_radians_);
    tilt_rate_rad_s_ = std::max(
      0.05, declare_parameter<double>("tilt_rate_deg_s", 45.0) * degrees_to_radians_);
    pan_min_rad_ = declare_parameter<double>("pan_min_deg", -180.0) * degrees_to_radians_;
    pan_max_rad_ = declare_parameter<double>("pan_max_deg", 180.0) * degrees_to_radians_;
    tilt_min_rad_ = declare_parameter<double>("tilt_min_deg", -90.0) * degrees_to_radians_;
    tilt_max_rad_ = declare_parameter<double>("tilt_max_deg", 90.0) * degrees_to_radians_;
    joy_timeout_s_ = std::max(0.1, declare_parameter<double>("joy_timeout", 0.5));
    if (pan_min_rad_ > pan_max_rad_) std::swap(pan_min_rad_, pan_max_rad_);
    if (tilt_min_rad_ > tilt_max_rad_) std::swap(tilt_min_rad_, tilt_max_rad_);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&GimbalControlNode::onJoy, this, _1));
    pan_pub_ = create_publisher<std_msgs::msg::Float64>("/gimbal/cmd_pan", 10);
    tilt_pub_ = create_publisher<std_msgs::msg::Float64>("/gimbal/cmd_tilt", 10);
    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&GimbalControlNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "Gimbal control ready: D-pad left/right pan, up/down tilt");
  }

private:
  enum Button : std::size_t
  {
    DPAD_UP = 11,
    DPAD_DOWN = 12,
    DPAD_LEFT = 13,
    DPAD_RIGHT = 14
  };

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr message)
  {
    buttons_ = message->buttons;
    last_joy_ = now();
    have_joy_ = true;
  }

  bool button(std::size_t index) const
  {
    return index < buttons_.size() && buttons_[index] != 0;
  }

  bool joyFresh() const
  {
    return have_joy_ && (now() - last_joy_).seconds() <= joy_timeout_s_;
  }

  void onTimer()
  {
    const auto stamp = now();
    const double dt = std::clamp((stamp - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = stamp;
    if (joyFresh()) {
      const double pan_input = static_cast<double>(button(DPAD_LEFT)) -
        static_cast<double>(button(DPAD_RIGHT));
      const double tilt_input = static_cast<double>(button(DPAD_DOWN)) -
        static_cast<double>(button(DPAD_UP));
      pan_target_rad_ = std::clamp(
        pan_target_rad_ + pan_input * pan_rate_rad_s_ * dt,
        pan_min_rad_, pan_max_rad_);
      tilt_target_rad_ = std::clamp(
        tilt_target_rad_ + tilt_input * tilt_rate_rad_s_ * dt,
        tilt_min_rad_, tilt_max_rad_);
    }

    std_msgs::msg::Float64 pan;
    pan.data = pan_target_rad_;
    pan_pub_->publish(pan);
    std_msgs::msg::Float64 tilt;
    tilt.data = tilt_target_rad_;
    tilt_pub_->publish(tilt);
  }

  static constexpr double degrees_to_radians_ = 0.017453292519943295;
  double pan_rate_rad_s_ {1.0471975512};
  double tilt_rate_rad_s_ {0.7853981634};
  double pan_min_rad_ {-3.1415926536};
  double pan_max_rad_ {3.1415926536};
  double tilt_min_rad_ {-1.5707963268};
  double tilt_max_rad_ {1.5707963268};
  double joy_timeout_s_ {0.5};
  double pan_target_rad_ {0.0};
  double tilt_target_rad_ {0.0};
  std::vector<int32_t> buttons_;
  bool have_joy_ {false};
  rclcpp::Time last_joy_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_ {0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pan_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalControlNode>());
  rclcpp::shutdown();
  return 0;
}
