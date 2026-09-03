#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <drone_interfaces/msg/swarm_drone_state.hpp>
#include <drone_interfaces/msg/swarm_state.hpp>
#include <drone_interfaces/srv/swarm_gimbal_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace drone_swarm
{

using SwarmDroneState = drone_interfaces::msg::SwarmDroneState;
using SwarmState = drone_interfaces::msg::SwarmState;
using SwarmGimbalCommand = drone_interfaces::srv::SwarmGimbalCommand;
using Float64 = std_msgs::msg::Float64;
using Bool = std_msgs::msg::Bool;

class SwarmGimbalRouterNode : public rclcpp::Node
{
public:
  SwarmGimbalRouterNode()
  : Node("swarm_gimbal_router")
  {
    command_rate_hz_ = std::max(
      1.0, declare_parameter<double>("command_rate_hz", 25.0));
    pan_rate_rad_s_ = std::max(
      0.05, declare_parameter<double>("pan_rate_deg_s", 60.0) * degrees_to_radians_);
    tilt_rate_rad_s_ = std::max(
      0.05, declare_parameter<double>("tilt_rate_deg_s", 45.0) * degrees_to_radians_);
    pan_min_rad_ = declare_parameter<double>("pan_min_deg", -180.0) * degrees_to_radians_;
    pan_max_rad_ = declare_parameter<double>("pan_max_deg", 180.0) * degrees_to_radians_;
    tilt_min_rad_ = declare_parameter<double>("tilt_min_deg", -90.0) * degrees_to_radians_;
    tilt_max_rad_ = declare_parameter<double>("tilt_max_deg", 90.0) * degrees_to_radians_;
    home_tolerance_rad_ = std::max(
      0.01, declare_parameter<double>("home_tolerance_deg", 1.0) * degrees_to_radians_);
    home_timeout_s_ = std::max(
      0.5, declare_parameter<double>("home_timeout_sec", 10.0));
    if (pan_min_rad_ > pan_max_rad_) std::swap(pan_min_rad_, pan_max_rad_);
    if (tilt_min_rad_ > tilt_max_rad_) std::swap(tilt_min_rad_, tilt_max_rad_);

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_sub_ = create_subscription<SwarmState>(
      "/swarm/state", state_qos,
      std::bind(&SwarmGimbalRouterNode::onSwarmState, this, std::placeholders::_1));
    command_service_ = create_service<SwarmGimbalCommand>(
      "/swarm/gimbal_command",
      std::bind(
        &SwarmGimbalRouterNode::handleCommand, this,
        std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / command_rate_hz_)),
      std::bind(&SwarmGimbalRouterNode::onTimer, this));
    last_tick_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "Swarm gimbal router ready: %.1f Hz, routing only to registered drones with has_gimbal=true",
      command_rate_hz_);
  }

private:
  struct DroneChannel
  {
    std::string id;
    std::string drone_namespace;
    bool registered {false};
    bool connected {false};
    bool has_gimbal {false};
    bool operator_enabled {false};
    bool safety_excluded {false};
    bool have_joint_state {false};
    bool swarm_output_active {false};
    double pan_rad {0.0};
    double tilt_rad {0.0};
    double commanded_pan_rad {0.0};
    double commanded_tilt_rad {0.0};
    rclcpp::Publisher<Float64>::SharedPtr pan_pub;
    rclcpp::Publisher<Float64>::SharedPtr tilt_pub;
    rclcpp::Publisher<Bool>::SharedPtr active_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  };

  static std::string normalizeNamespace(std::string value)
  {
    if (value.empty() || value == "/") return "";
    if (value.front() != '/') value.insert(value.begin(), '/');
    while (value.size() > 1 && value.back() == '/') value.pop_back();
    return value;
  }

  static bool isDirectional(uint8_t command)
  {
    return command == SwarmGimbalCommand::Request::COMMAND_UP ||
           command == SwarmGimbalCommand::Request::COMMAND_DOWN ||
           command == SwarmGimbalCommand::Request::COMMAND_LEFT ||
           command == SwarmGimbalCommand::Request::COMMAND_RIGHT;
  }

  static bool isKnownCommand(uint8_t command)
  {
    return isDirectional(command) ||
           command == SwarmGimbalCommand::Request::COMMAND_HOME ||
           command == SwarmGimbalCommand::Request::COMMAND_STOP;
  }

  std::shared_ptr<DroneChannel> ensureChannelLocked(
    const SwarmDroneState & state)
  {
    const auto found = channels_.find(state.drone_id);
    if (found != channels_.end()) {
      if (found->second->drone_namespace != normalizeNamespace(state.drone_namespace)) {
        RCLCPP_WARN(
          get_logger(), "Ignoring namespace change for gimbal drone %s", state.drone_id.c_str());
      }
      return found->second;
    }

    const auto drone_namespace = normalizeNamespace(state.drone_namespace);
    if (state.drone_id.empty() || drone_namespace.empty()) return nullptr;

    auto channel = std::make_shared<DroneChannel>();
    channel->id = state.drone_id;
    channel->drone_namespace = drone_namespace;
    channel->pan_pub = create_publisher<Float64>(
      drone_namespace + "/gimbal/swarm_cmd_pan", 10);
    channel->tilt_pub = create_publisher<Float64>(
      drone_namespace + "/gimbal/swarm_cmd_tilt", 10);
    channel->active_pub = create_publisher<Bool>(
      drone_namespace + "/gimbal/swarm_control_active", 10);
    channel->joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      drone_namespace + "/gimbal/joint_state", rclcpp::SensorDataQoS(),
      [this, channel](const sensor_msgs::msg::JointState::SharedPtr message) {
        onJointState(channel, message);
      });
    channels_.emplace(channel->id, channel);
    RCLCPP_INFO(
      get_logger(), "Discovered gimbal routing channels for %s at %s",
      channel->id.c_str(), channel->drone_namespace.c_str());
    return channel;
  }

  void onSwarmState(const SwarmState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & entry : channels_) entry.second->connected = false;
    for (const auto & state : message->drones) {
      const auto channel = ensureChannelLocked(state);
      if (!channel) continue;
      channel->registered = state.registered;
      channel->connected = state.connected;
      channel->has_gimbal = state.has_gimbal;
      channel->operator_enabled = state.operator_enabled;
      channel->safety_excluded = state.safety_excluded;
      if (!eligibleLocked(*channel) && channel->swarm_output_active) {
        publishActiveLocked(*channel, false);
      }
    }
  }

  void onJointState(
    const std::shared_ptr<DroneChannel> & channel,
    const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      if (index >= message->position.size()) break;
      const auto & name = message->name[index];
      if (name == "gimbal_pan_joint" || name == "pan_joint" || name == "pan") {
        channel->pan_rad = message->position[index];
        channel->have_joint_state = true;
      } else if (
        name == "gimbal_pitch_joint" || name == "tilt_joint" || name == "tilt")
      {
        channel->tilt_rad = message->position[index];
        channel->have_joint_state = true;
      }
    }
    if (!channel->swarm_output_active) {
      channel->commanded_pan_rad = channel->pan_rad;
      channel->commanded_tilt_rad = channel->tilt_rad;
    }
  }

  bool eligibleLocked(const DroneChannel & channel) const
  {
    return channel.registered && channel.connected && channel.has_gimbal &&
           channel.operator_enabled && !channel.safety_excluded;
  }

  std::vector<std::shared_ptr<DroneChannel>> selectedChannelsLocked(
    const SwarmGimbalCommand::Request & request,
    std::string & error)
  {
    std::vector<std::shared_ptr<DroneChannel>> selected;
    if (request.target_mode == SwarmGimbalCommand::Request::TARGET_ALL) {
      std::size_t ignored_without_gimbal = 0;
      for (const auto & entry : channels_) {
        if (eligibleLocked(*entry.second)) {
          selected.push_back(entry.second);
        } else if (
          entry.second->registered && entry.second->connected && !entry.second->has_gimbal)
        {
          ++ignored_without_gimbal;
        }
      }
      if (ignored_without_gimbal > 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "ALL gimbal command ignored for %zu connected registered drone(s) without a gimbal",
          ignored_without_gimbal);
      }
      if (selected.empty()) error = "no connected registered drone with gimbal capability";
      return selected;
    }
    if (request.target_mode != SwarmGimbalCommand::Request::TARGET_DRONE) {
      error = "invalid gimbal target mode";
      return selected;
    }
    const auto found = channels_.find(request.drone_id);
    if (found == channels_.end()) {
      error = "unknown drone: " + request.drone_id;
      return selected;
    }
    if (!found->second->has_gimbal) {
      error = request.drone_id + " does not report a gimbal";
      return selected;
    }
    if (!eligibleLocked(*found->second)) {
      error = request.drone_id + " is not connected and controllable";
      return selected;
    }
    selected.push_back(found->second);
    return selected;
  }

  void prepareForSwarmLocked(DroneChannel & channel)
  {
    if (channel.swarm_output_active) return;
    if (channel.have_joint_state) {
      channel.commanded_pan_rad = channel.pan_rad;
      channel.commanded_tilt_rad = channel.tilt_rad;
    }
  }

  void publishActiveLocked(DroneChannel & channel, bool active)
  {
    Bool message;
    message.data = active;
    channel.active_pub->publish(message);
    channel.swarm_output_active = active;
  }

  void publishTargetLocked(DroneChannel & channel, bool active)
  {
    Float64 pan;
    pan.data = channel.commanded_pan_rad;
    channel.pan_pub->publish(pan);
    Float64 tilt;
    tilt.data = channel.commanded_tilt_rad;
    channel.tilt_pub->publish(tilt);
    publishActiveLocked(channel, active);
  }

  void stopChannelLocked(DroneChannel & channel)
  {
    if (channel.have_joint_state) {
      channel.commanded_pan_rad = channel.pan_rad;
      channel.commanded_tilt_rad = channel.tilt_rad;
    }
    publishTargetLocked(channel, false);
  }

  void clearSpecificCommandLocked(const std::string & drone_id)
  {
    held_commands_.erase(drone_id);
    home_targets_.erase(drone_id);
    home_started_.erase(drone_id);
  }

  void stopSelectedLocked(
    const std::vector<std::shared_ptr<DroneChannel>> & selected)
  {
    for (const auto & channel : selected) stopChannelLocked(*channel);
  }

  void handleCommand(
    const SwarmGimbalCommand::Request::SharedPtr request,
    SwarmGimbalCommand::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->accepted = false;
    response->routed_drone_count = 0;
    if (!isKnownCommand(request->command)) {
      response->message = "unknown gimbal command";
      return;
    }
    std::string error;
    const auto selected = selectedChannelsLocked(*request, error);
    if (!error.empty()) {
      response->message = error;
      return;
    }
    response->routed_drone_count = static_cast<uint32_t>(selected.size());

    const bool target_all = request->target_mode == SwarmGimbalCommand::Request::TARGET_ALL;
    if (request->command == SwarmGimbalCommand::Request::COMMAND_HOME) {
      if (target_all) {
        all_held_command_.reset();
        held_commands_.clear();
      } else {
        clearSpecificCommandLocked(request->drone_id);
      }
      const auto started = std::chrono::steady_clock::now();
      for (const auto & channel : selected) {
        channel->commanded_pan_rad = 0.0;
        channel->commanded_tilt_rad = 0.0;
        home_targets_.insert(channel->id);
        home_started_[channel->id] = started;
        publishTargetLocked(*channel, true);
      }
      response->accepted = true;
      response->message = "home command routed to " +
        std::to_string(selected.size()) + " gimbal(s)";
      return;
    }

    if (request->command == SwarmGimbalCommand::Request::COMMAND_STOP ||
      (isDirectional(request->command) && !request->pressed))
    {
      if (target_all) {
        all_held_command_.reset();
        held_commands_.clear();
        home_targets_.clear();
        home_started_.clear();
      } else {
        clearSpecificCommandLocked(request->drone_id);
      }
      stopSelectedLocked(selected);
      response->accepted = true;
      response->message = "stop command routed to " +
        std::to_string(selected.size()) + " gimbal(s)";
      return;
    }

    if (!request->pressed) {
      response->message = "directional gimbal commands require pressed=true or STOP";
      return;
    }

    if (target_all) {
      all_held_command_ = request->command;
      held_commands_.clear();
      for (const auto & channel : selected) {
        home_targets_.erase(channel->id);
        home_started_.erase(channel->id);
        prepareForSwarmLocked(*channel);
        publishTargetLocked(*channel, true);
      }
    } else {
      held_commands_[request->drone_id] = request->command;
      home_targets_.erase(request->drone_id);
      home_started_.erase(request->drone_id);
      prepareForSwarmLocked(*selected.front());
      publishTargetLocked(*selected.front(), true);
    }
    response->accepted = true;
    response->message = "gimbal command routed to " +
      std::to_string(selected.size()) + " gimbal(s)";
  }

  std::optional<uint8_t> commandForChannelLocked(const std::string & drone_id) const
  {
    const auto specific = held_commands_.find(drone_id);
    if (specific != held_commands_.end()) return specific->second;
    return all_held_command_;
  }

  void applyJogLocked(DroneChannel & channel, uint8_t command, double dt_s)
  {
    const double pan_step = pan_rate_rad_s_ * dt_s;
    const double tilt_step = tilt_rate_rad_s_ * dt_s;
    if (command == SwarmGimbalCommand::Request::COMMAND_LEFT) {
      channel.commanded_pan_rad += pan_step;
    } else if (command == SwarmGimbalCommand::Request::COMMAND_RIGHT) {
      channel.commanded_pan_rad -= pan_step;
    } else if (command == SwarmGimbalCommand::Request::COMMAND_UP) {
      channel.commanded_tilt_rad -= tilt_step;
    } else if (command == SwarmGimbalCommand::Request::COMMAND_DOWN) {
      channel.commanded_tilt_rad += tilt_step;
    }
    channel.commanded_pan_rad = std::clamp(
      channel.commanded_pan_rad, pan_min_rad_, pan_max_rad_);
    channel.commanded_tilt_rad = std::clamp(
      channel.commanded_tilt_rad, tilt_min_rad_, tilt_max_rad_);
    publishTargetLocked(channel, true);
  }

  void updateHomeLocked(
    const std::shared_ptr<DroneChannel> & channel,
    const std::chrono::steady_clock::time_point & current_time)
  {
    channel->commanded_pan_rad = 0.0;
    channel->commanded_tilt_rad = 0.0;
    publishTargetLocked(*channel, true);
    const bool reached = channel->have_joint_state &&
      std::fabs(channel->pan_rad) <= home_tolerance_rad_ &&
      std::fabs(channel->tilt_rad) <= home_tolerance_rad_;
    const auto started = home_started_.find(channel->id);
    const bool timed_out = started != home_started_.end() &&
      std::chrono::duration<double>(current_time - started->second).count() >= home_timeout_s_;
    if (reached || timed_out) {
      if (timed_out && !reached) {
        RCLCPP_WARN(
          get_logger(), "Gimbal HOME timeout for %s; holding its current command",
          channel->id.c_str());
      }
      home_targets_.erase(channel->id);
      home_started_.erase(channel->id);
      stopChannelLocked(*channel);
    }
  }

  void onTimer()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt_s = std::clamp(
      std::chrono::duration<double>(current_time - last_tick_).count(), 0.0, 0.2);
    last_tick_ = current_time;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : channels_) {
      const auto & channel = entry.second;
      if (!eligibleLocked(*channel)) {
        if (channel->swarm_output_active) publishActiveLocked(*channel, false);
        continue;
      }
      if (home_targets_.count(channel->id) != 0) {
        updateHomeLocked(channel, current_time);
        continue;
      }
      const auto command = commandForChannelLocked(channel->id);
      if (command) {
        prepareForSwarmLocked(*channel);
        applyJogLocked(*channel, *command, dt_s);
      } else if (channel->swarm_output_active) {
        stopChannelLocked(*channel);
      }
    }
  }

  static constexpr double degrees_to_radians_ = 0.017453292519943295;
  double command_rate_hz_ {25.0};
  double pan_rate_rad_s_ {1.0471975512};
  double tilt_rate_rad_s_ {0.7853981634};
  double pan_min_rad_ {-3.1415926536};
  double pan_max_rad_ {3.1415926536};
  double tilt_min_rad_ {-1.5707963268};
  double tilt_max_rad_ {1.5707963268};
  double home_tolerance_rad_ {0.0174532925};
  double home_timeout_s_ {10.0};
  std::map<std::string, std::shared_ptr<DroneChannel>> channels_;
  std::optional<uint8_t> all_held_command_;
  std::map<std::string, uint8_t> held_commands_;
  std::set<std::string> home_targets_;
  std::map<std::string, std::chrono::steady_clock::time_point> home_started_;
  std::chrono::steady_clock::time_point last_tick_;
  std::mutex mutex_;
  rclcpp::Subscription<SwarmState>::SharedPtr state_sub_;
  rclcpp::Service<SwarmGimbalCommand>::SharedPtr command_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_swarm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_swarm::SwarmGimbalRouterNode>());
  rclcpp::shutdown();
  return 0;
}
