#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <drone_interfaces/msg/motion_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using MotionCommand = drone_interfaces::msg::MotionCommand;
using std::placeholders::_1;

class CommandMuxNode : public rclcpp::Node
{
public:
  CommandMuxNode()
  : Node("command_mux")
  {
    command_timeout_s_ = std::max(
      0.1, declare_parameter<double>("command_timeout_s", 0.5));
    manual_sub_ = create_subscription<MotionCommand>(
      "/motion/manual_intent", 10,
      [this](const MotionCommand::SharedPtr message) {manual_.set(*message, now());});
    autonomous_sub_ = create_subscription<MotionCommand>(
      "/motion/autonomous_intent", 10,
      [this](const MotionCommand::SharedPtr message) {autonomous_.set(*message, now());});
    supervisor_sub_ = create_subscription<MotionCommand>(
      "/motion/supervisor_intent", 10,
      [this](const MotionCommand::SharedPtr message) {supervisor_.set(*message, now());});
    selected_pub_ = create_publisher<MotionCommand>("/motion/selected_intent", 10);
    override_pub_ = create_publisher<std_msgs::msg::Bool>("/navigation/manual_override", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&CommandMuxNode::onTimer, this));
  }

private:
  struct Input
  {
    MotionCommand command;
    rclcpp::Time received {0, 0, RCL_ROS_TIME};
    bool available {false};

    void set(const MotionCommand & value, const rclcpp::Time & stamp)
    {
      command = value;
      received = stamp;
      available = true;
    }
  };

  bool fresh(const Input & input) const
  {
    return input.available && input.command.active &&
      (now() - input.received).seconds() <= command_timeout_s_;
  }

  void onTimer()
  {
    const bool manual_fresh = fresh(manual_);
    const bool manual_override = manual_fresh && manual_.command.manual_override;
    const MotionCommand * selected = nullptr;
    if (fresh(supervisor_)) {
      selected = &supervisor_.command;
    } else if (manual_override) {
      selected = &manual_.command;
    } else if (fresh(autonomous_)) {
      selected = &autonomous_.command;
    } else if (manual_fresh) {
      selected = &manual_.command;
    }

    MotionCommand output;
    output.header.stamp = now();
    output.header.frame_id = "map";
    if (selected != nullptr) {
      output = *selected;
      output.header.stamp = now();
    }
    selected_pub_->publish(output);

    std_msgs::msg::Bool override_message;
    override_message.data = manual_override;
    override_pub_->publish(override_message);
  }

  double command_timeout_s_ {0.5};
  Input manual_;
  Input autonomous_;
  Input supervisor_;
  rclcpp::Subscription<MotionCommand>::SharedPtr manual_sub_;
  rclcpp::Subscription<MotionCommand>::SharedPtr autonomous_sub_;
  rclcpp::Subscription<MotionCommand>::SharedPtr supervisor_sub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr selected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandMuxNode>());
  rclcpp::shutdown();
  return 0;
}
