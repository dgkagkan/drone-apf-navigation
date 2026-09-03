#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace
{

using Float64 = std_msgs::msg::Float64;
using Bool = std_msgs::msg::Bool;

class GimbalCommandMuxNode : public rclcpp::Node
{
public:
  GimbalCommandMuxNode()
  : Node("gimbal_command_mux")
  {
    publish_rate_hz_ = std::max(
      1.0, declare_parameter<double>("publish_rate_hz", 50.0));
    swarm_timeout_s_ = std::max(
      0.1, declare_parameter<double>("swarm_command_timeout_sec", 0.75));
    stop_hold_s_ = std::max(
      0.0, declare_parameter<double>("stop_hold_sec", 0.75));

    teleop_pan_sub_ = create_subscription<Float64>(
      "gimbal/teleop_cmd_pan", 10,
      [this](const Float64::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (swarm_hold_after_stop_ && have_teleop_pan_ &&
          std::fabs(message->data - teleop_pan_) > 1e-6)
        {
          swarm_hold_after_stop_ = false;
        }
        teleop_pan_ = message->data;
        have_teleop_pan_ = true;
      });
    teleop_tilt_sub_ = create_subscription<Float64>(
      "gimbal/teleop_cmd_tilt", 10,
      [this](const Float64::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (swarm_hold_after_stop_ && have_teleop_tilt_ &&
          std::fabs(message->data - teleop_tilt_) > 1e-6)
        {
          swarm_hold_after_stop_ = false;
        }
        teleop_tilt_ = message->data;
        have_teleop_tilt_ = true;
      });
    swarm_pan_sub_ = create_subscription<Float64>(
      "gimbal/swarm_cmd_pan", 10,
      [this](const Float64::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        swarm_pan_ = message->data;
        have_swarm_pan_ = true;
        last_swarm_command_ = std::chrono::steady_clock::now();
      });
    swarm_tilt_sub_ = create_subscription<Float64>(
      "gimbal/swarm_cmd_tilt", 10,
      [this](const Float64::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        swarm_tilt_ = message->data;
        have_swarm_tilt_ = true;
        last_swarm_command_ = std::chrono::steady_clock::now();
      });
    swarm_active_sub_ = create_subscription<Bool>(
      "gimbal/swarm_control_active", 10,
      [this](const Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        swarm_active_ = message->data;
        if (message->data) {
          swarm_hold_after_stop_ = false;
          last_swarm_command_ = std::chrono::steady_clock::now();
        } else {
          stop_hold_until_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(stop_hold_s_));
          swarm_hold_after_stop_ = true;
        }
      });

    pan_pub_ = create_publisher<Float64>("gimbal/cmd_pan", 10);
    tilt_pub_ = create_publisher<Float64>("gimbal/cmd_tilt", 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&GimbalCommandMuxNode::onTimer, this));

    last_swarm_command_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "Gimbal command mux ready: joystick source is default, swarm source has priority while active");
  }

private:
  void onTimer()
  {
    double pan = 0.0;
    double tilt = 0.0;
    bool publish_pan = false;
    bool publish_tilt = false;
    bool warn_timeout = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto current_time = std::chrono::steady_clock::now();
      const bool swarm_fresh = std::chrono::duration<double>(
          current_time - last_swarm_command_).count() <= swarm_timeout_s_;
      const bool stop_hold = current_time < stop_hold_until_;
      const bool use_swarm = (swarm_active_ && swarm_fresh) ||
        stop_hold || swarm_hold_after_stop_;
      if (swarm_active_ && !swarm_fresh) {
        swarm_active_ = false;
        swarm_hold_after_stop_ = true;
        warn_timeout = true;
      }

      if (use_swarm) {
        if (have_swarm_pan_) {
          pan = swarm_pan_;
          publish_pan = true;
        }
        if (have_swarm_tilt_) {
          tilt = swarm_tilt_;
          publish_tilt = true;
        }
      } else {
        if (have_teleop_pan_) {
          pan = teleop_pan_;
          publish_pan = true;
        }
        if (have_teleop_tilt_) {
          tilt = teleop_tilt_;
          publish_tilt = true;
        }
      }
    }

    if (warn_timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Swarm gimbal command timed out; returning control to joystick source");
    }
    if (publish_pan) {
      Float64 message;
      message.data = pan;
      pan_pub_->publish(message);
    }
    if (publish_tilt) {
      Float64 message;
      message.data = tilt;
      tilt_pub_->publish(message);
    }
  }

  double publish_rate_hz_ {50.0};
  double swarm_timeout_s_ {0.75};
  double stop_hold_s_ {0.75};
  double teleop_pan_ {0.0};
  double teleop_tilt_ {0.0};
  double swarm_pan_ {0.0};
  double swarm_tilt_ {0.0};
  bool have_teleop_pan_ {false};
  bool have_teleop_tilt_ {false};
  bool have_swarm_pan_ {false};
  bool have_swarm_tilt_ {false};
  bool swarm_active_ {false};
  bool swarm_hold_after_stop_ {false};
  std::chrono::steady_clock::time_point last_swarm_command_;
  std::chrono::steady_clock::time_point stop_hold_until_;
  std::mutex mutex_;
  rclcpp::Subscription<Float64>::SharedPtr teleop_pan_sub_;
  rclcpp::Subscription<Float64>::SharedPtr teleop_tilt_sub_;
  rclcpp::Subscription<Float64>::SharedPtr swarm_pan_sub_;
  rclcpp::Subscription<Float64>::SharedPtr swarm_tilt_sub_;
  rclcpp::Subscription<Bool>::SharedPtr swarm_active_sub_;
  rclcpp::Publisher<Float64>::SharedPtr pan_pub_;
  rclcpp::Publisher<Float64>::SharedPtr tilt_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalCommandMuxNode>());
  rclcpp::shutdown();
  return 0;
}
