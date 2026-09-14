#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <drone_interfaces/action/navigate_to.hpp>
#include <drone_interfaces/msg/apf_telemetry.hpp>
#include <drone_interfaces/msg/flight_request.hpp>
#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <drone_interfaces/srv/validate_goal.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

using ApfTelemetry = drone_interfaces::msg::ApfTelemetry;
using FlightRequest = drone_interfaces::msg::FlightRequest;
using MotionCommand = drone_interfaces::msg::MotionCommand;
using NavigateTo = drone_interfaces::action::NavigateTo;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateTo>;
using ValidateGoal = drone_interfaces::srv::ValidateGoal;
using VehicleState = drone_interfaces::msg::VehicleState;
using std::placeholders::_1;
using std::placeholders::_2;

class NavigationServerNode : public rclcpp::Node
{
public:
  NavigationServerNode()
  : Node("navigation_server")
  {
    cube_size_m_ = std::max(
      1.0, declare_parameter<double>("geofence.cube_size_m", 2000.0));
    min_altitude_m_ = declare_parameter<double>("geofence.min_altitude_m", 2.0);
    no_fly_zone_values_ = declare_parameter<std::vector<double>>(
      "geofence.no_fly_zones", std::vector<double>{});
    goal_tolerance_m_ = std::max(
      0.2, declare_parameter<double>("goal_tolerance_m", 25.0));
    altitude_tolerance_m_ = std::max(
      0.2, declare_parameter<double>("altitude_tolerance_m", 2.0));
    goal_chain_grace_period_s_ = std::max(
      0.05, declare_parameter<double>("goal_chain_grace_period_s", 0.25));
    default_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("default_speed_m_s", 15.0));
    max_speed_m_s_ = std::max(
      default_speed_m_s_, declare_parameter<double>("max_speed_m_s", 20.0));
    altitude_gain_ = std::max(
      0.1, declare_parameter<double>("altitude_gain", 0.8));
    multicopter_arrival_gain_ = std::max(
      0.1, declare_parameter<double>("multicopter_arrival_gain", 0.8));
    max_vertical_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("max_vertical_speed_m_s", 3.0));
    transition_request_period_s_ = std::max(
      0.2, declare_parameter<double>("transition_request_period_s", 1.0));

    if (no_fly_zone_values_.size() % 6 != 0) {
      RCLCPP_ERROR(
        get_logger(), "geofence.no_fly_zones must contain groups of 6 values; ignoring it");
      no_fly_zone_values_.clear();
    }

    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&NavigationServerNode::onState, this, _1));
    flight_command_sub_ = create_subscription<FlightRequest>(
      "/flight/request", 10,
      std::bind(&NavigationServerNode::onFlightRequest, this, _1));
    telemetry_sub_ = create_subscription<ApfTelemetry>(
      "/apf/telemetry", 10,
      [this](const ApfTelemetry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        avoidance_active_ = message->avoidance_active;
        if (!message->active_mode.empty()) active_apf_mode_ = message->active_mode;
      });
    override_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/navigation/manual_override", 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        manual_override_ = message->data;
      });
    speed_override_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/navigation/speed_override", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        has_speed_override_ = std::isfinite(message->data) && message->data > 0.0;
        speed_override_m_s_ = has_speed_override_ ? message->data : 0.0;
        if (has_speed_override_) {
          RCLCPP_INFO(
            get_logger(), "Navigation speed override set to %.1f m/s", speed_override_m_s_);
        } else {
          RCLCPP_INFO(get_logger(), "Navigation speed override cleared");
        }
      });
    command_pub_ = create_publisher<MotionCommand>("/motion/autonomous_intent", 10);
    flight_request_pub_ = create_publisher<FlightRequest>("/flight/request", 10);
    active_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/navigation/active_goal",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    validate_service_ = create_service<ValidateGoal>(
      "/navigation/validate_goal",
      std::bind(&NavigationServerNode::validateGoalService, this, _1, _2));
    action_server_ = rclcpp_action::create_server<NavigateTo>(
      this, "/navigate_to",
      std::bind(&NavigationServerNode::handleGoal, this, _1, _2),
      std::bind(&NavigationServerNode::handleCancel, this, _1),
      std::bind(&NavigationServerNode::handleAccepted, this, _1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&NavigationServerNode::onTimer, this));
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&NavigationServerNode::onParametersSet, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Navigation action ready; %.0fx%.0fx%.0fm geofence will be centered at startup",
      cube_size_m_, cube_size_m_, cube_size_m_);
  }

private:
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      double goal_tolerance = goal_tolerance_m_;
      double altitude_tolerance = altitude_tolerance_m_;
      double goal_chain_grace = goal_chain_grace_period_s_;
      double default_speed = default_speed_m_s_;
      double max_speed = max_speed_m_s_;
      double altitude_gain = altitude_gain_;
      double arrival_gain = multicopter_arrival_gain_;
      double max_vertical_speed = max_vertical_speed_m_s_;
      double transition_period = transition_request_period_s_;
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "goal_tolerance_m") goal_tolerance = parameter.as_double();
        else if (name == "altitude_tolerance_m") altitude_tolerance = parameter.as_double();
        else if (name == "goal_chain_grace_period_s") {
          goal_chain_grace = parameter.as_double();
        } else if (name == "default_speed_m_s") {
          default_speed = parameter.as_double();
        } else if (name == "max_speed_m_s") {
          max_speed = parameter.as_double();
        } else if (name == "altitude_gain") {
          altitude_gain = parameter.as_double();
        } else if (name == "multicopter_arrival_gain") {
          arrival_gain = parameter.as_double();
        } else if (name == "max_vertical_speed_m_s") {
          max_vertical_speed = parameter.as_double();
        } else if (name == "transition_request_period_s") {
          transition_period = parameter.as_double();
        }
      }
      if (!std::isfinite(goal_tolerance) || goal_tolerance < 0.2 || goal_tolerance > 200.0) {
        result.successful = false;
        result.reason = "goal tolerance must be between 0.2 and 200 m";
      } else if (!std::isfinite(altitude_tolerance) || altitude_tolerance < 0.2 ||
        altitude_tolerance > 50.0)
      {
        result.successful = false;
        result.reason = "altitude tolerance must be between 0.2 and 50 m";
      } else if (!std::isfinite(goal_chain_grace) || goal_chain_grace < 0.05 ||
        goal_chain_grace > 10.0)
      {
        result.successful = false;
        result.reason = "goal chain grace period must be between 0.05 and 10 s";
      } else if (!std::isfinite(default_speed) || default_speed < 0.2 ||
        default_speed > max_speed)
      {
        result.successful = false;
        result.reason = "default speed must be between 0.2 m/s and the maximum speed";
      } else if (!std::isfinite(max_speed) || max_speed < 0.2 || max_speed > 100.0 ||
        max_speed < default_speed)
      {
        result.successful = false;
        result.reason = "maximum speed must be at least the default speed and at most 100 m/s";
      } else if (!std::isfinite(altitude_gain) || altitude_gain < 0.1 || altitude_gain > 10.0) {
        result.successful = false;
        result.reason = "altitude gain must be between 0.1 and 10";
      } else if (!std::isfinite(arrival_gain) || arrival_gain < 0.1 || arrival_gain > 10.0) {
        result.successful = false;
        result.reason = "multicopter arrival gain must be between 0.1 and 10";
      } else if (!std::isfinite(max_vertical_speed) || max_vertical_speed < 0.2 ||
        max_vertical_speed > 30.0)
      {
        result.successful = false;
        result.reason = "vertical speed must be between 0.2 and 30 m/s";
      } else if (!std::isfinite(transition_period) || transition_period < 0.2 ||
        transition_period > 10.0)
      {
        result.successful = false;
        result.reason = "transition request period must be between 0.2 and 10 s";
      }
      if (result.successful) {
        goal_tolerance_m_ = goal_tolerance;
        altitude_tolerance_m_ = altitude_tolerance;
        goal_chain_grace_period_s_ = goal_chain_grace;
        default_speed_m_s_ = default_speed;
        max_speed_m_s_ = max_speed;
        altitude_gain_ = altitude_gain;
        multicopter_arrival_gain_ = arrival_gain;
        max_vertical_speed_m_s_ = max_vertical_speed;
        transition_request_period_s_ = transition_period;
      }
    } catch (const std::exception & exception) {
      result.successful = false;
      result.reason = exception.what();
    }
    if (result.successful) {
      RCLCPP_INFO(get_logger(), "Runtime navigation parameter update accepted");
    }
    return result;
  }

  struct GoalValidation
  {
    bool valid;
    std::string reason;
  };

  GoalValidation validateTarget(const geometry_msgs::msg::PoseStamped & target) const
  {
    const auto & point = target.pose.position;
    if (!target.header.frame_id.empty() && target.header.frame_id != "map") {
      return {false, "target frame must be map"};
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return {false, "target contains a non-finite coordinate"};
    }
    if (!geofence_origin_initialized_) {
      return {false, "startup geofence origin is not available yet"};
    }
    const double half_size = 0.5 * cube_size_m_;
    if (std::fabs(point.x - geofence_origin_x_) > half_size ||
      std::fabs(point.y - geofence_origin_y_) > half_size ||
      std::fabs(point.z - geofence_origin_z_) > half_size)
    {
      return {false, "target is outside the 2000x2000x2000m startup geofence"};
    }
    if (point.z < min_altitude_m_) {
      return {false, "target altitude is below the minimum flight altitude"};
    }
    for (std::size_t index = 0; index < no_fly_zone_values_.size(); index += 6) {
      if (point.x >= no_fly_zone_values_[index] &&
        point.x <= no_fly_zone_values_[index + 1] &&
        point.y >= no_fly_zone_values_[index + 2] &&
        point.y <= no_fly_zone_values_[index + 3] &&
        point.z >= no_fly_zone_values_[index + 4] &&
        point.z <= no_fly_zone_values_[index + 5])
      {
        return {false, "target is inside a no-fly zone"};
      }
    }
    return {true, "goal is valid"};
  }

  void validateGoalService(
    const ValidateGoal::Request::SharedPtr request,
    ValidateGoal::Response::SharedPtr response)
  {
    const auto validation = validateTarget(request->target);
    response->valid = validation.valid;
    response->reason = validation.reason;
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const NavigateTo::Goal> goal)
  {
    const auto validation = validateTarget(goal->target);
    if (!validation.valid) {
      RCLCPP_WARN(get_logger(), "Rejected navigation goal: %s", validation.reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_state_ || !state_.position_valid) {
      RCLCPP_WARN(get_logger(), "Rejected navigation goal: vehicle position is invalid");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleNavigate>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    std::shared_ptr<GoalHandleNavigate> previous;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      previous = active_goal_;
      active_goal_ = goal_handle;
      last_transition_request_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      arrival_hold_pending_ = false;
      arrival_hold_active_ = false;
      hold_transition_requested_ = false;
      suppress_cancel_hold_ = false;
    }
    if (previous && previous->is_active()) {
      auto result = std::make_shared<NavigateTo::Result>();
      result->result_code = NavigateTo::Result::ABORTED;
      result->message = "preempted by a newer goal";
      previous->abort(result);
    }
    RCLCPP_INFO(
      get_logger(), "Accepted goal ENU=(%.1f, %.1f, %.1f)",
      goal_handle->get_goal()->target.pose.position.x,
      goal_handle->get_goal()->target.pose.position.y,
      goal_handle->get_goal()->target.pose.position.z);
    auto target = goal_handle->get_goal()->target;
    target.header.stamp = now();
    target.header.frame_id = "map";
    active_goal_pub_->publish(target);
  }

  void onState(const VehicleState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = *message;
    have_state_ = true;
    if (!geofence_origin_initialized_ && message->position_valid) {
      geofence_origin_x_ = message->position_enu.x;
      geofence_origin_y_ = message->position_enu.y;
      geofence_origin_z_ = message->position_enu.z;
      geofence_origin_initialized_ = true;
      RCLCPP_INFO(
        get_logger(), "Geofence centered at ENU=(%.1f, %.1f, %.1f)",
        geofence_origin_x_, geofence_origin_y_, geofence_origin_z_);
    }
  }

  void onFlightRequest(const FlightRequest::SharedPtr request)
  {
    if (request->request != FlightRequest::LAND &&
      request->request != FlightRequest::DISARM &&
      request->request != FlightRequest::FORCE_DISARM)
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      suppress_cancel_hold_ = true;
      arrival_hold_pending_ = false;
      arrival_hold_active_ = false;
      hold_transition_requested_ = false;
    }
    publishInactive();
    RCLCPP_INFO(
      get_logger(), "Flight termination requested; autonomous altitude hold released");
  }

  void publishInactive()
  {
    MotionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "map";
    command.source = MotionCommand::SOURCE_AUTONOMOUS;
    command_pub_->publish(command);
  }

  void publishArrivalHold(double altitude_m)
  {
    MotionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "map";
    command.vehicle_mode = MotionCommand::MODE_MULTICOPTER;
    command.source = MotionCommand::SOURCE_AUTONOMOUS;
    command.active = true;
    command.hold_altitude = true;
    command.target_altitude_m = altitude_m;
    command_pub_->publish(command);
  }

  void scheduleArrivalHold(double altitude_m, bool immediate = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arrival_hold_altitude_m_ = altitude_m;
    arrival_hold_start_ = now();
    arrival_hold_pending_ = !immediate;
    arrival_hold_active_ = immediate;
    hold_transition_requested_ = false;
  }

  bool transitionRequestDue()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto current_time = now();
    if ((current_time - last_transition_request_).seconds() < transition_request_period_s_) {
      return false;
    }
    last_transition_request_ = current_time;
    return true;
  }

  void processArrivalHold(const VehicleState & state)
  {
    bool activated = false;
    bool active = false;
    bool request_transition = false;
    double altitude_m = state.position_enu.z;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (arrival_hold_pending_ &&
        (now() - arrival_hold_start_).seconds() >= goal_chain_grace_period_s_)
      {
        arrival_hold_pending_ = false;
        arrival_hold_active_ = true;
        activated = true;
      }
      if (arrival_hold_active_ && !hold_transition_requested_ &&
        state.vehicle_mode != VehicleState::MODE_MULTICOPTER)
      {
        hold_transition_requested_ = true;
        request_transition = true;
      }
      active = arrival_hold_active_;
      altitude_m = arrival_hold_altitude_m_;
    }

    if (activated) {
      RCLCPP_INFO(
        get_logger(), "Goal chain complete; holding in MC at %.1fm", altitude_m);
    }
    if (request_transition) {
      FlightRequest request;
      request.header.stamp = now();
      request.request = FlightRequest::TRANSITION_TO_MC;
      flight_request_pub_->publish(request);
    }
    if (active) {
      publishArrivalHold(altitude_m);
    } else {
      publishInactive();
    }
  }

  void finishGoal(
    const std::shared_ptr<GoalHandleNavigate> & goal_handle,
    uint8_t result_code, const std::string & message, double final_distance,
    bool canceled)
  {
    auto result = std::make_shared<NavigateTo::Result>();
    result->result_code = result_code;
    result->message = message;
    result->final_distance_m = final_distance;
    if (canceled && goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (result_code == NavigateTo::Result::SUCCEEDED) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    publishInactive();
  }

  void onTimer()
  {
    std::shared_ptr<GoalHandleNavigate> goal_handle;
    VehicleState state;
    bool manual_override = false;
    bool avoidance_active = false;
    std::string active_apf_mode;
    bool has_speed_override = false;
    double speed_override_m_s = 0.0;
    double goal_tolerance_m = 25.0;
    double configured_altitude_tolerance_m = 2.0;
    double default_speed_m_s = 15.0;
    double max_speed_m_s = 20.0;
    double altitude_gain = 0.8;
    double multicopter_arrival_gain = 0.8;
    double max_vertical_speed_m_s = 3.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle = active_goal_;
      state = state_;
      manual_override = manual_override_;
      avoidance_active = avoidance_active_;
      active_apf_mode = active_apf_mode_;
      has_speed_override = has_speed_override_;
      speed_override_m_s = speed_override_m_s_;
      goal_tolerance_m = goal_tolerance_m_;
      configured_altitude_tolerance_m = altitude_tolerance_m_;
      default_speed_m_s = default_speed_m_s_;
      max_speed_m_s = max_speed_m_s_;
      altitude_gain = altitude_gain_;
      multicopter_arrival_gain = multicopter_arrival_gain_;
      max_vertical_speed_m_s = max_vertical_speed_m_s_;
    }
    if (!goal_handle || !goal_handle->is_active()) {
      processArrivalHold(state);
      return;
    }

    const auto goal = goal_handle->get_goal();
    const auto & target = goal->target.pose.position;
    const double east = target.x - state.position_enu.x;
    const double north = target.y - state.position_enu.y;
    const double up = target.z - state.position_enu.z;
    const double horizontal_distance = std::hypot(east, north);
    const double distance = std::sqrt(horizontal_distance * horizontal_distance + up * up);

    if (goal_handle->is_canceling()) {
      bool enter_arrival_hold = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        enter_arrival_hold = !suppress_cancel_hold_;
      }
      if (enter_arrival_hold) {
        scheduleArrivalHold(state.position_enu.z, true);
        RCLCPP_WARN(
          get_logger(), "Navigation canceled; entering MC altitude hold at %.1fm",
          state.position_enu.z);
      } else {
        RCLCPP_INFO(
          get_logger(), "Navigation canceled for LAND/disarm; altitude hold stays inactive");
      }
      finishGoal(
        goal_handle, NavigateTo::Result::CANCELED, "goal canceled", distance, true);
      return;
    }
    if (manual_override) {
      publishInactive();
      auto feedback = std::make_shared<NavigateTo::Feedback>();
      feedback->phase = "paused by manual controller";
      feedback->remaining_distance_m = distance;
      feedback->avoidance_active = avoidance_active;
      feedback->apf_mode = active_apf_mode;
      feedback->position_enu = state.position_enu;
      feedback->velocity_enu = state.velocity_enu;
      feedback->speed_m_s = std::sqrt(
        state.velocity_enu.x * state.velocity_enu.x +
        state.velocity_enu.y * state.velocity_enu.y +
        state.velocity_enu.z * state.velocity_enu.z);
      goal_handle->publish_feedback(feedback);
      return;
    }
    const auto validation = validateTarget(goal->target);
    if (!validation.valid) {
      finishGoal(
        goal_handle, NavigateTo::Result::ABORTED,
        "goal became invalid: " + validation.reason, distance, false);
      return;
    }
    const double horizontal_tolerance_m = goal->horizontal_tolerance_m > 0.0 ?
      std::max(0.2, goal->horizontal_tolerance_m) : goal_tolerance_m;
    const double altitude_tolerance_m = goal->altitude_tolerance_m > 0.0 ?
      std::max(0.2, goal->altitude_tolerance_m) : configured_altitude_tolerance_m;
    if (horizontal_distance <= horizontal_tolerance_m &&
      std::fabs(up) <= altitude_tolerance_m)
    {
      scheduleArrivalHold(state.position_enu.z);
      finishGoal(
        goal_handle, NavigateTo::Result::SUCCEEDED, "goal reached", distance, false);
      return;
    }

    const uint8_t requested_vehicle_mode = goal->use_fixed_wing ?
      VehicleState::MODE_FIXED_WING : VehicleState::MODE_MULTICOPTER;
    if (state.vehicle_mode != requested_vehicle_mode && transitionRequestDue())
    {
      FlightRequest request;
      request.header.stamp = now();
      request.request = goal->use_fixed_wing ?
        FlightRequest::TRANSITION_TO_FW : FlightRequest::TRANSITION_TO_MC;
      flight_request_pub_->publish(request);
      RCLCPP_INFO(
        get_logger(), "Requesting VTOL transition to %s",
        goal->use_fixed_wing ? "fixed wing" : "multicopter");
    }

    const double requested_speed = has_speed_override ? speed_override_m_s :
      (goal->cruise_speed_m_s > 0.0 ? goal->cruise_speed_m_s : default_speed_m_s);
    const double speed = std::clamp(requested_speed, 0.2, max_speed_m_s);
    const double horizontal_speed = goal->use_fixed_wing ? speed :
      std::min(speed, multicopter_arrival_gain * horizontal_distance);
    MotionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "map";
    command.vehicle_mode = goal->use_fixed_wing ?
      MotionCommand::MODE_FIXED_WING : MotionCommand::MODE_MULTICOPTER;
    command.source = MotionCommand::SOURCE_AUTONOMOUS;
    command.active = true;
    command.hold_altitude = true;
    command.target_altitude_m = target.z;
    if (horizontal_distance > 0.05) {
      command.velocity_enu.x = horizontal_speed * east / horizontal_distance;
      command.velocity_enu.y = horizontal_speed * north / horizontal_distance;
    }
    if (!goal->use_fixed_wing) {
      command.velocity_enu.z = std::clamp(
        altitude_gain * up, -max_vertical_speed_m_s, max_vertical_speed_m_s);
    }
    command_pub_->publish(command);

    auto feedback = std::make_shared<NavigateTo::Feedback>();
    if (state.vehicle_mode != requested_vehicle_mode) {
      feedback->phase = goal->use_fixed_wing ?
        "transitioning to fixed wing" : "transitioning to multicopter";
    } else {
      feedback->phase = goal->use_fixed_wing ?
        "fixed-wing cruise" : "multicopter navigation";
    }
    feedback->remaining_distance_m = distance;
    feedback->avoidance_active = avoidance_active;
    feedback->apf_mode = active_apf_mode;
    feedback->position_enu = state.position_enu;
    feedback->velocity_enu = state.velocity_enu;
    feedback->speed_m_s = std::sqrt(
      state.velocity_enu.x * state.velocity_enu.x +
      state.velocity_enu.y * state.velocity_enu.y +
      state.velocity_enu.z * state.velocity_enu.z);
    goal_handle->publish_feedback(feedback);
  }

  double cube_size_m_ {2000.0};
  double min_altitude_m_ {2.0};
  double geofence_origin_x_ {0.0};
  double geofence_origin_y_ {0.0};
  double geofence_origin_z_ {0.0};
  std::vector<double> no_fly_zone_values_;
  double goal_tolerance_m_ {25.0};
  double altitude_tolerance_m_ {2.0};
  double goal_chain_grace_period_s_ {0.25};
  double default_speed_m_s_ {15.0};
  double max_speed_m_s_ {20.0};
  double altitude_gain_ {0.8};
  double multicopter_arrival_gain_ {0.8};
  double max_vertical_speed_m_s_ {3.0};
  double transition_request_period_s_ {1.0};
  std::mutex mutex_;
  VehicleState state_;
  bool have_state_ {false};
  bool manual_override_ {false};
  bool avoidance_active_ {false};
  bool has_speed_override_ {false};
  double speed_override_m_s_ {0.0};
  std::string active_apf_mode_ {"unknown"};
  rclcpp::Time last_transition_request_ {0, 0, RCL_ROS_TIME};
  bool arrival_hold_pending_ {false};
  bool arrival_hold_active_ {false};
  bool hold_transition_requested_ {false};
  bool suppress_cancel_hold_ {false};
  bool geofence_origin_initialized_ {false};
  double arrival_hold_altitude_m_ {0.0};
  rclcpp::Time arrival_hold_start_ {0, 0, RCL_ROS_TIME};
  std::shared_ptr<GoalHandleNavigate> active_goal_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<FlightRequest>::SharedPtr flight_command_sub_;
  rclcpp::Subscription<ApfTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_override_sub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<FlightRequest>::SharedPtr flight_request_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr active_goal_pub_;
  rclcpp::Service<ValidateGoal>::SharedPtr validate_service_;
  rclcpp_action::Server<NavigateTo>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationServerNode>());
  rclcpp::shutdown();
  return 0;
}
