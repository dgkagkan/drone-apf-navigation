#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <drone_interfaces/action/navigate_to.hpp>
#include <drone_interfaces/action/takeoff.hpp>
#include <drone_interfaces/msg/flight_request.hpp>
#include <drone_interfaces/msg/route_target.hpp>
#include <drone_interfaces/msg/swarm_mission_command.hpp>
#include <drone_interfaces/msg/swarm_mission_feedback.hpp>
#include <drone_interfaces/msg/swarm_route.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>

using NavigateTo = drone_interfaces::action::NavigateTo;
using Takeoff = drone_interfaces::action::Takeoff;
using FlightRequest = drone_interfaces::msg::FlightRequest;
using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateTo>;
using RouteTarget = drone_interfaces::msg::RouteTarget;
using SwarmMissionCommand = drone_interfaces::msg::SwarmMissionCommand;
using SwarmMissionFeedback = drone_interfaces::msg::SwarmMissionFeedback;
using SwarmRoute = drone_interfaces::msg::SwarmRoute;

namespace drone_navigation
{

class RouteExecutorNode : public rclcpp::Node
{
public:
  RouteExecutorNode()
  : Node("route_executor")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    drone_id_ = declare_parameter<std::string>("drone_id", "");
    if (drone_id_.empty()) drone_id_ = droneIdFromNamespace();
    if (drone_id_.empty()) {
      throw std::runtime_error("route_executor requires drone_id or a non-root namespace");
    }
    navigate_server_wait_log_period_ms_ = std::max<int64_t>(
      1000, declare_parameter<int64_t>("navigate_server_wait_log_period_ms", 5000));

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    const auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    mission_command_sub_ = create_subscription<SwarmMissionCommand>(
      "/swarm/mission_command", command_qos,
      std::bind(&RouteExecutorNode::onMissionCommand, this, std::placeholders::_1));
    flight_command_sub_ = create_subscription<SwarmMissionCommand>(
      "/swarm/mission_command", rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&RouteExecutorNode::onFlightCommand, this, std::placeholders::_1));
    mission_feedback_pub_ = create_publisher<SwarmMissionFeedback>(
      "/swarm/mission_feedback", feedback_qos);
    speed_override_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/navigation/speed_override", rclcpp::QoS(1).reliable().transient_local());
    lidar_range_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/perception/lidar_range_override", rclcpp::QoS(1).reliable().transient_local());
    flight_request_pub_ = create_publisher<FlightRequest>("/flight/request", 10);
    navigate_client_ = rclcpp_action::create_client<NavigateTo>(this, "/navigate_to");
    takeoff_client_ = rclcpp_action::create_client<Takeoff>(this, "/takeoff");
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RouteExecutorNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Route executor %s ready: /swarm/mission_command -> sequential /navigate_to goals",
      drone_id_.c_str());
  }

private:
  std::string droneIdFromNamespace() const
  {
    std::string node_namespace = get_namespace();
    while (node_namespace.size() > 1 && node_namespace.back() == '/') {
      node_namespace.pop_back();
    }
    const auto separator = node_namespace.find_last_of('/');
    if (separator == std::string::npos) return node_namespace;
    return node_namespace.substr(separator + 1);
  }

  bool routeTargetValid(const RouteTarget & route_target) const
  {
    const auto & point = route_target.target.pose.position;
    return (route_target.target.header.frame_id.empty() ||
           route_target.target.header.frame_id == map_frame_) &&
           std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }

  std::pair<bool, std::string> validateRoute(const SwarmRoute & route) const
  {
    if (route.route_id == 0) return {false, "route_id must be non-zero"};
    if (route.targets.empty()) return {false, "route contains no targets"};
    std::set<uint64_t> target_ids;
    for (const auto & target : route.targets) {
      if (!routeTargetValid(target)) {
        return {false, "target " + std::to_string(target.target_id) +
                 " is not a finite " + map_frame_ + "-frame target"};
      }
      if (!target_ids.insert(target.target_id).second) {
        return {false, "route contains duplicate target IDs"};
      }
    }
    return {true, "route is valid"};
  }

  bool commandTargetsThisDrone(const SwarmMissionCommand & command) const
  {
    return command.target_drone_ids.empty() ||
           std::find(
      command.target_drone_ids.begin(), command.target_drone_ids.end(), drone_id_) !=
           command.target_drone_ids.end();
  }

  const SwarmRoute * findOwnRoute(const SwarmMissionCommand & command) const
  {
    const auto route = std::find_if(
      command.routes.begin(), command.routes.end(),
      [this](const SwarmRoute & candidate) {return candidate.drone_id == drone_id_;});
    return route == command.routes.end() ? nullptr : &*route;
  }

  SwarmMissionFeedback makeFeedbackLocked(
    uint8_t state,
    const std::string & message,
    double remaining_distance_m = std::numeric_limits<double>::quiet_NaN()) const
  {
    SwarmMissionFeedback feedback;
    feedback.header.stamp = now();
    feedback.header.frame_id = map_frame_;
    feedback.command_id = command_id_;
    feedback.mission_id = mission_id_;
    feedback.revision = revision_;
    feedback.drone_id = drone_id_;
    feedback.route_id = route_id_;
    feedback.state = state;
    feedback.stored_target_count = route_targets_.size();
    feedback.active_target_index = active_target_index_;
    feedback.completed_target_count = completed_target_count_;
    feedback.remaining_distance_m = remaining_distance_m;
    feedback.message = message;
    if (active_target_index_ < route_targets_.size()) {
      feedback.active_target_id = route_targets_[active_target_index_].target_id;
    }
    return feedback;
  }

  void publishFeedback(
    uint8_t state,
    const std::string & message,
    double remaining_distance_m = std::numeric_limits<double>::quiet_NaN())
  {
    SwarmMissionFeedback feedback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      feedback = makeFeedbackLocked(state, message, remaining_distance_m);
      last_feedback_state_ = state;
      last_feedback_message_ = message;
    }
    mission_feedback_pub_->publish(feedback);
  }

  void onMissionCommand(const SwarmMissionCommand::SharedPtr command)
  {
    if (command->command == SwarmMissionCommand::SET_SPEED) {
      if (commandTargetsThisDrone(*command)) {
        std_msgs::msg::Float64 speed_override;
        speed_override.data = command->cruise_speed_m_s;
        speed_override_pub_->publish(speed_override);
        if (speed_override.data > 0.0) {
          RCLCPP_INFO(
            get_logger(), "Applied broadcast speed override %.1f m/s", speed_override.data);
        } else {
          RCLCPP_INFO(get_logger(), "Cleared broadcast speed override");
        }
      }
      return;
    }
    if (command->command == SwarmMissionCommand::SET_LIDAR_RANGE) {
      if (commandTargetsThisDrone(*command)) {
        std_msgs::msg::Float64 lidar_range;
        lidar_range.data = command->lidar_range_m;
        lidar_range_pub_->publish(lidar_range);
        RCLCPP_INFO(
          get_logger(), "Applied broadcast LiDAR/APF range %.1f m", lidar_range.data);
      }
      return;
    }
    if (command->command == SwarmMissionCommand::LAND ||
      command->command == SwarmMissionCommand::ARM ||
      command->command == SwarmMissionCommand::TAKEOFF) return;
    if (command->command == SwarmMissionCommand::CANCEL) {
      handleCancelCommand(*command);
      return;
    }
    if (command->command != SwarmMissionCommand::EXECUTE) {
      RCLCPP_WARN(get_logger(), "Ignoring unknown swarm command %u", command->command);
      return;
    }

    const auto * own_route = findOwnRoute(*command);
    if (!own_route) {
      cancelRouteRemovedByRevision(*command);
      return;
    }

    const auto validation = validateRoute(*own_route);
    if (!validation.first) {
      SwarmMissionFeedback feedback;
      feedback.header.stamp = now();
      feedback.header.frame_id = map_frame_;
      feedback.command_id = command->command_id;
      feedback.mission_id = command->mission_id;
      feedback.revision = command->revision;
      feedback.drone_id = drone_id_;
      feedback.route_id = own_route->route_id;
      feedback.state = SwarmMissionFeedback::REJECTED;
      feedback.stored_target_count = own_route->targets.size();
      feedback.remaining_distance_m = std::numeric_limits<double>::quiet_NaN();
      feedback.message = validation.second;
      mission_feedback_pub_->publish(feedback);
      RCLCPP_WARN(
        get_logger(), "Rejected broadcast route %lu: %s",
        own_route->route_id, validation.second.c_str());
      return;
    }

    NavigateGoalHandle::SharedPtr previous_child;
    bool duplicate = false;
    bool stale = false;
    bool unchanged = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      duplicate = command->command_id == command_id_;
      stale = command->mission_id < mission_id_ ||
        (command->mission_id == mission_id_ && command->revision < revision_);
      unchanged = route_active_ && command->mission_id == mission_id_ &&
        own_route->route_id == route_id_;
      if (!duplicate && !stale) {
        command_id_ = command->command_id;
        mission_id_ = command->mission_id;
        revision_ = command->revision;
        if (!unchanged) {
          previous_child = active_navigate_goal_;
          ++generation_;
          route_id_ = own_route->route_id;
          route_targets_ = own_route->targets;
          active_target_index_ = 0;
          completed_target_count_ = 0;
          navigate_request_in_progress_ = false;
          active_navigate_goal_.reset();
          cancel_requested_ = false;
          route_active_ = true;
        }
      }
    }

    if (stale) return;
    if (duplicate) {
      publishFeedback(last_feedback_state_, last_feedback_message_);
      return;
    }
    if (previous_child) navigate_client_->async_cancel_goal(previous_child);

    publishFeedback(
      SwarmMissionFeedback::ACCEPTED,
      unchanged ? "unchanged route kept running" : "broadcast route stored and accepted");
    RCLCPP_INFO(
      get_logger(), "%s route %lu from mission %lu revision %u with %zu target(s)",
      unchanged ? "Kept" : "Accepted", own_route->route_id, command->mission_id,
      command->revision, own_route->targets.size());
  }

  void cancelRouteRemovedByRevision(const SwarmMissionCommand & command)
  {
    NavigateGoalHandle::SharedPtr child;
    bool should_cancel = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      should_cancel = route_active_ && command.mission_id == mission_id_ &&
        command.revision > revision_;
      if (should_cancel) {
        command_id_ = command.command_id;
        revision_ = command.revision;
        cancel_requested_ = true;
        child = active_navigate_goal_;
      }
    }
    if (!should_cancel) return;
    if (child) navigate_client_->async_cancel_goal(child);
    RCLCPP_INFO(
      get_logger(), "Canceling route removed by mission %lu revision %u",
      command.mission_id, command.revision);
  }

  void handleCancelCommand(const SwarmMissionCommand & command)
  {
    NavigateGoalHandle::SharedPtr child;
    bool accepted = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      accepted = route_active_ && command.mission_id == mission_id_ &&
        command.revision >= revision_ && commandTargetsThisDrone(command);
      if (accepted) {
        command_id_ = command.command_id;
        revision_ = command.revision;
        cancel_requested_ = true;
        child = active_navigate_goal_;
      }
    }
    if (!accepted) return;
    if (child) navigate_client_->async_cancel_goal(child);
    RCLCPP_INFO(
      get_logger(), "Accepted broadcast cancel for mission %lu", command.mission_id);
  }

  void handleLandCommand(const SwarmMissionCommand & command)
  {
    if (!commandTargetsThisDrone(command)) return;

    NavigateGoalHandle::SharedPtr child;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_id_ = command.command_id;
      if (route_active_) {
        cancel_requested_ = true;
        child = active_navigate_goal_;
      }
    }
    if (child) navigate_client_->async_cancel_goal(child);

    FlightRequest request;
    request.header.stamp = now();
    request.header.frame_id = map_frame_;
    request.request = FlightRequest::LAND;
    flight_request_pub_->publish(request);
    RCLCPP_WARN(get_logger(), "Accepted broadcast LAND command");
  }

  void onFlightCommand(const SwarmMissionCommand::SharedPtr command)
  {
    if (!commandTargetsThisDrone(*command)) return;
    switch (command->command) {
      case SwarmMissionCommand::ARM:
        publishFlightRequest(FlightRequest::ARM_OFFBOARD);
        RCLCPP_INFO(get_logger(), "Accepted broadcast ARM command");
        break;
      case SwarmMissionCommand::TAKEOFF:
        sendTakeoffGoal(*command);
        break;
      case SwarmMissionCommand::LAND:
        handleLandCommand(*command);
        break;
      default:
        break;
    }
  }

  void publishFlightRequest(uint8_t request_type)
  {
    FlightRequest request;
    request.header.stamp = now();
    request.header.frame_id = map_frame_;
    request.request = request_type;
    flight_request_pub_->publish(request);
  }

  void sendTakeoffGoal(const SwarmMissionCommand & command)
  {
    if (!takeoff_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "Rejected broadcast TAKEOFF: local action server unavailable");
      return;
    }
    Takeoff::Goal goal;
    goal.target_altitude_m = command.takeoff_altitude_m;
    goal.climb_speed_m_s = command.takeoff_climb_speed_m_s;
    rclcpp_action::Client<Takeoff>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<Takeoff>::SharedPtr & goal_handle) {
        RCLCPP_INFO(
          get_logger(), "Broadcast TAKEOFF %s by local flight supervisor",
          goal_handle ? "accepted" : "rejected");
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<Takeoff>::WrappedResult & result) {
        const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        RCLCPP_INFO(
          get_logger(), "Broadcast TAKEOFF %s", succeeded ? "completed" : "failed");
      };
    takeoff_client_->async_send_goal(goal, options);
  }

  void onTimer()
  {
    RouteTarget route_target;
    uint64_t generation = 0;
    bool send_target = false;
    bool finish_canceled = false;
    bool finish_succeeded = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!route_active_) return;
      if (cancel_requested_ && !navigate_request_in_progress_ && !active_navigate_goal_) {
        finish_canceled = true;
        generation = generation_;
      } else if (!cancel_requested_ && active_target_index_ >= route_targets_.size()) {
        finish_succeeded = true;
        generation = generation_;
      } else if (cancel_requested_ || navigate_request_in_progress_ || active_navigate_goal_) {
        return;
      } else if (!navigate_client_->action_server_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), navigate_server_wait_log_period_ms_,
          "Waiting for the local /navigate_to action server");
        return;
      } else {
        route_target = route_targets_[active_target_index_];
        generation = generation_;
        navigate_request_in_progress_ = true;
        send_target = true;
      }
    }

    if (finish_canceled) {
      finishRoute(generation, SwarmMissionFeedback::CANCELED, "route canceled");
    } else if (finish_succeeded) {
      finishRoute(generation, SwarmMissionFeedback::SUCCEEDED, "route completed");
    } else if (send_target) {
      sendNavigateGoal(route_target, generation);
    }
  }

  void sendNavigateGoal(const RouteTarget & route_target, uint64_t generation)
  {
    NavigateTo::Goal goal;
    goal.target = route_target.target;
    goal.cruise_speed_m_s = route_target.cruise_speed_m_s;
    goal.use_fixed_wing = route_target.use_fixed_wing;

    rclcpp_action::Client<NavigateTo>::SendGoalOptions options;
    options.goal_response_callback =
      [this, generation, target_id = route_target.target_id](
      const NavigateGoalHandle::SharedPtr & handle)
      {
        bool stale = false;
        bool rejected = false;
        bool cancel = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stale = generation != generation_ || !route_active_;
          if (!stale) {
            navigate_request_in_progress_ = false;
            active_navigate_goal_ = handle;
            rejected = !handle;
            cancel = cancel_requested_ && static_cast<bool>(handle);
          }
        }
        if (stale && handle) {
          navigate_client_->async_cancel_goal(handle);
        } else if (rejected) {
          finishRoute(
            generation, SwarmMissionFeedback::FAILED,
            "local navigation server rejected target " + std::to_string(target_id));
        } else if (cancel) {
          navigate_client_->async_cancel_goal(handle);
        } else {
          RCLCPP_INFO(get_logger(), "Executing broadcast target %lu", target_id);
        }
      };
    options.feedback_callback =
      [this, generation](
      NavigateGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateTo::Feedback> feedback)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation != generation_ || !route_active_) return;
        }
        publishFeedback(
          SwarmMissionFeedback::EXECUTING, feedback->phase,
          feedback->remaining_distance_m);
      };
    options.result_callback =
      [this, generation, target_id = route_target.target_id](
      const NavigateGoalHandle::WrappedResult & result)
      {
        bool succeeded = false;
        bool canceled = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation != generation_ || !route_active_) return;
          active_navigate_goal_.reset();
          succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->result_code == NavigateTo::Result::SUCCEEDED;
          canceled = cancel_requested_;
          if (succeeded) {
            ++active_target_index_;
            ++completed_target_count_;
          }
        }
        if (succeeded) {
          RCLCPP_INFO(get_logger(), "Completed broadcast target %lu", target_id);
        } else if (canceled) {
          finishRoute(generation, SwarmMissionFeedback::CANCELED, "route canceled");
        } else {
          const std::string message = result.result ? result.result->message : "no result";
          finishRoute(
            generation, SwarmMissionFeedback::FAILED,
            "target " + std::to_string(target_id) + " failed: " + message);
        }
      };
    navigate_client_->async_send_goal(goal, options);
  }

  void finishRoute(uint64_t generation, uint8_t state, const std::string & message)
  {
    SwarmMissionFeedback feedback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != generation_ || !route_active_) return;
      feedback = makeFeedbackLocked(state, message);
      last_feedback_state_ = state;
      last_feedback_message_ = message;
      route_active_ = false;
      route_targets_.clear();
      active_target_index_ = 0;
      completed_target_count_ = 0;
      navigate_request_in_progress_ = false;
      active_navigate_goal_.reset();
      cancel_requested_ = false;
    }
    mission_feedback_pub_->publish(feedback);
    RCLCPP_INFO(get_logger(), "Broadcast route finished: %s", message.c_str());
  }

  std::mutex mutex_;
  std::string map_frame_ {"map"};
  std::string drone_id_;
  int64_t navigate_server_wait_log_period_ms_ {5000};
  uint64_t generation_ {0};
  uint64_t command_id_ {0};
  uint64_t mission_id_ {0};
  uint32_t revision_ {0};
  uint64_t route_id_ {0};
  std::vector<RouteTarget> route_targets_;
  std::size_t active_target_index_ {0};
  uint32_t completed_target_count_ {0};
  uint8_t last_feedback_state_ {SwarmMissionFeedback::RECEIVED};
  std::string last_feedback_message_ {"waiting for mission"};
  bool route_active_ {false};
  bool navigate_request_in_progress_ {false};
  bool cancel_requested_ {false};
  NavigateGoalHandle::SharedPtr active_navigate_goal_;
  rclcpp_action::Client<NavigateTo>::SharedPtr navigate_client_;
  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp::Subscription<SwarmMissionCommand>::SharedPtr mission_command_sub_;
  rclcpp::Subscription<SwarmMissionCommand>::SharedPtr flight_command_sub_;
  rclcpp::Publisher<SwarmMissionFeedback>::SharedPtr mission_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_override_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lidar_range_pub_;
  rclcpp::Publisher<FlightRequest>::SharedPtr flight_request_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_navigation::RouteExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
