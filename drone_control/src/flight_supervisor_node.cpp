#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <drone_interfaces/action/land.hpp>
#include <drone_interfaces/action/takeoff.hpp>
#include <drone_interfaces/action/transition_vtol.hpp>
#include <drone_interfaces/msg/flight_request.hpp>
#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <drone_interfaces/srv/arm.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using Arm = drone_interfaces::srv::Arm;
using FlightRequest = drone_interfaces::msg::FlightRequest;
using Land = drone_interfaces::action::Land;
using MotionCommand = drone_interfaces::msg::MotionCommand;
using Takeoff = drone_interfaces::action::Takeoff;
using TransitionVtol = drone_interfaces::action::TransitionVtol;
using VehicleState = drone_interfaces::msg::VehicleState;
using VC = px4_msgs::msg::VehicleCommand;
using LandGoalHandle = rclcpp_action::ServerGoalHandle<Land>;
using TakeoffGoalHandle = rclcpp_action::ServerGoalHandle<Takeoff>;
using TransitionGoalHandle = rclcpp_action::ServerGoalHandle<TransitionVtol>;
using std::placeholders::_1;
using std::placeholders::_2;

class FlightSupervisorNode : public rclcpp::Node
{
public:
  FlightSupervisorNode()
  : Node("flight_supervisor")
  {
    takeoff_tolerance_m_ = std::max(
      0.1, declare_parameter<double>("takeoff_tolerance_m", 0.5));
    landed_altitude_m_ = std::max(
      0.05, declare_parameter<double>("landed_altitude_m", 0.3));
    command_period_s_ = std::max(
      0.2, declare_parameter<double>("vehicle_command_period_s", 1.0));

    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&FlightSupervisorNode::onState, this, _1));
    request_sub_ = create_subscription<FlightRequest>(
      "/flight/request", 10,
      std::bind(&FlightSupervisorNode::onFlightRequest, this, _1));
    command_pub_ = create_publisher<VC>("/px4/vehicle_command_request", 10);
    intent_pub_ = create_publisher<MotionCommand>("/motion/supervisor_intent", 10);
    arm_service_ = create_service<Arm>(
      "/flight/arm", std::bind(&FlightSupervisorNode::onArmService, this, _1, _2));

    takeoff_server_ = rclcpp_action::create_server<Takeoff>(
      this, "/takeoff",
      std::bind(&FlightSupervisorNode::handleTakeoffGoal, this, _1, _2),
      std::bind(&FlightSupervisorNode::handleTakeoffCancel, this, _1),
      std::bind(&FlightSupervisorNode::handleTakeoffAccepted, this, _1));
    land_server_ = rclcpp_action::create_server<Land>(
      this, "/land",
      std::bind(&FlightSupervisorNode::handleLandGoal, this, _1, _2),
      std::bind(&FlightSupervisorNode::handleLandCancel, this, _1),
      std::bind(&FlightSupervisorNode::handleLandAccepted, this, _1));
    transition_server_ = rclcpp_action::create_server<TransitionVtol>(
      this, "/transition_vtol",
      std::bind(&FlightSupervisorNode::handleTransitionGoal, this, _1, _2),
      std::bind(&FlightSupervisorNode::handleTransitionCancel, this, _1),
      std::bind(&FlightSupervisorNode::handleTransitionAccepted, this, _1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&FlightSupervisorNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "Flight supervisor ready: arm, takeoff, land, VTOL transition");
  }

private:
  enum class Operation
  {
    IDLE,
    TAKEOFF,
    LAND,
    TRANSITION
  };

  bool operationIdle() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return operation_ == Operation::IDLE;
  }

  void onState(const VehicleState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = *message;
    have_state_ = true;
  }

  void sendCommand(uint32_t command, float parameter1 = 0.0F, float parameter2 = 0.0F)
  {
    VC message;
    message.command = command;
    message.param1 = parameter1;
    message.param2 = parameter2;
    command_pub_->publish(message);
  }

  bool commandDue()
  {
    if ((now() - last_vehicle_command_).seconds() < command_period_s_) return false;
    last_vehicle_command_ = now();
    return true;
  }

  void requestArmOffboard()
  {
    arm_offboard_requested_ = true;
  }

  void publishInactiveIntent()
  {
    MotionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "map";
    command.source = MotionCommand::SOURCE_SUPERVISOR;
    intent_pub_->publish(command);
  }

  void publishTakeoffIntent(double target_altitude_m, double climb_speed_m_s)
  {
    MotionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "map";
    command.source = MotionCommand::SOURCE_SUPERVISOR;
    command.vehicle_mode = MotionCommand::MODE_MULTICOPTER;
    command.active = true;
    command.velocity_enu.z = std::max(0.2, climb_speed_m_s);
    command.hold_altitude = true;
    command.target_altitude_m = target_altitude_m;
    intent_pub_->publish(command);
  }

  void onArmService(const Arm::Request::SharedPtr request, Arm::Response::SharedPtr response)
  {
    if (request->arm) {
      requestArmOffboard();
      response->accepted = true;
      response->message = "arm and offboard requested";
    } else {
      arm_offboard_requested_ = false;
      sendCommand(VC::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
      response->accepted = true;
      response->message = "disarm requested";
    }
  }

  void onFlightRequest(const FlightRequest::SharedPtr request)
  {
    switch (request->request) {
      case FlightRequest::ARM_OFFBOARD:
        requestArmOffboard();
        break;
      case FlightRequest::DISARM:
        arm_offboard_requested_ = false;
        sendCommand(VC::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
        break;
      case FlightRequest::LAND:
        startManualLand();
        break;
      case FlightRequest::TRANSITION_TO_FW:
        startManualTransition(true);
        break;
      case FlightRequest::TRANSITION_TO_MC:
        startManualTransition(false);
        break;
      default:
        break;
    }
  }

  rclcpp_action::GoalResponse handleTakeoffGoal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const Takeoff::Goal> goal)
  {
    if (!std::isfinite(goal->target_altitude_m) || goal->target_altitude_m <= 0.5 ||
      !operationIdle())
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleTakeoffCancel(
    const std::shared_ptr<TakeoffGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleTakeoffAccepted(const std::shared_ptr<TakeoffGoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    operation_ = Operation::TAKEOFF;
    takeoff_goal_ = goal_handle;
    arm_offboard_requested_ = true;
    last_vehicle_command_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  rclcpp_action::GoalResponse handleLandGoal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const Land::Goal>)
  {
    return operationIdle() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
      rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handleLandCancel(const std::shared_ptr<LandGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleLandAccepted(const std::shared_ptr<LandGoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    operation_ = Operation::LAND;
    land_goal_ = goal_handle;
    manual_operation_ = false;
    last_vehicle_command_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  rclcpp_action::GoalResponse handleTransitionGoal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const TransitionVtol::Goal>)
  {
    return operationIdle() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
      rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handleTransitionCancel(
    const std::shared_ptr<TransitionGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleTransitionAccepted(const std::shared_ptr<TransitionGoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    operation_ = Operation::TRANSITION;
    transition_goal_ = goal_handle;
    transition_to_fw_ = goal_handle->get_goal()->fixed_wing;
    manual_operation_ = false;
    last_vehicle_command_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void startManualLand()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (operation_ != Operation::IDLE) return;
    operation_ = Operation::LAND;
    manual_operation_ = true;
    last_vehicle_command_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void startManualTransition(bool fixed_wing)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (operation_ != Operation::IDLE) return;
    operation_ = Operation::TRANSITION;
    transition_to_fw_ = fixed_wing;
    manual_operation_ = true;
    last_vehicle_command_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void processArmOffboard(const VehicleState & state)
  {
    if (!arm_offboard_requested_) return;
    if (state.armed && state.offboard) {
      arm_offboard_requested_ = false;
      RCLCPP_INFO(get_logger(), "Vehicle armed and Offboard active");
      return;
    }
    if (!commandDue()) return;
    sendCommand(VC::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
    if (!state.armed) sendCommand(VC::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
  }

  void processTakeoff(const VehicleState & state)
  {
    if (!takeoff_goal_) return;
    const auto goal = takeoff_goal_->get_goal();
    if (takeoff_goal_->is_canceling()) {
      auto result = std::make_shared<Takeoff::Result>();
      result->success = false;
      result->message = "takeoff canceled";
      result->final_altitude_m = state.position_enu.z;
      takeoff_goal_->canceled(result);
      finishOperation();
      return;
    }
    if (state.vehicle_mode != VehicleState::MODE_MULTICOPTER && commandDue()) {
      sendCommand(VC::VEHICLE_CMD_DO_VTOL_TRANSITION, 3.0F);
    }
    processArmOffboard(state);
    publishTakeoffIntent(goal->target_altitude_m, goal->climb_speed_m_s);

    auto feedback = std::make_shared<Takeoff::Feedback>();
    feedback->phase = state.armed && state.offboard ? "climbing" : "arming";
    feedback->current_altitude_m = state.position_enu.z;
    takeoff_goal_->publish_feedback(feedback);
    if (state.position_enu.z >= goal->target_altitude_m - takeoff_tolerance_m_) {
      auto result = std::make_shared<Takeoff::Result>();
      result->success = true;
      result->message = "takeoff complete";
      result->final_altitude_m = state.position_enu.z;
      takeoff_goal_->succeed(result);
      finishOperation();
    }
  }

  void processLand(const VehicleState & state)
  {
    if (land_goal_ && land_goal_->is_canceling()) {
      auto result = std::make_shared<Land::Result>();
      result->success = false;
      result->message = "landing canceled";
      land_goal_->canceled(result);
      requestArmOffboard();
      finishOperation();
      return;
    }
    if (state.vehicle_mode == VehicleState::MODE_FIXED_WING) {
      if (commandDue()) sendCommand(VC::VEHICLE_CMD_DO_VTOL_TRANSITION, 3.0F);
    } else if (commandDue()) {
      VC command;
      command.command = VC::VEHICLE_CMD_DO_SET_MODE;
      command.param1 = 1.0F;
      command.param2 = 4.0F;
      command.param3 = 6.0F;
      command_pub_->publish(command);
    }

    if (land_goal_) {
      auto feedback = std::make_shared<Land::Feedback>();
      feedback->phase = state.vehicle_mode == VehicleState::MODE_FIXED_WING ?
        "transitioning to multicopter" : "landing";
      feedback->current_altitude_m = state.position_enu.z;
      land_goal_->publish_feedback(feedback);
    }
    if (!state.armed || state.position_enu.z <= landed_altitude_m_) {
      if (land_goal_) {
        auto result = std::make_shared<Land::Result>();
        result->success = true;
        result->message = "landing complete";
        land_goal_->succeed(result);
      }
      finishOperation();
    }
  }

  void processTransition(const VehicleState & state)
  {
    if (transition_goal_ && transition_goal_->is_canceling()) {
      auto result = std::make_shared<TransitionVtol::Result>();
      result->success = false;
      result->message = "transition canceled";
      transition_goal_->canceled(result);
      finishOperation();
      return;
    }
    const uint8_t expected_mode = transition_to_fw_ ?
      VehicleState::MODE_FIXED_WING : VehicleState::MODE_MULTICOPTER;
    if (state.vehicle_mode == expected_mode) {
      if (transition_goal_) {
        auto result = std::make_shared<TransitionVtol::Result>();
        result->success = true;
        result->message = transition_to_fw_ ?
          "fixed-wing transition complete" : "multicopter transition complete";
        transition_goal_->succeed(result);
      }
      finishOperation();
      return;
    }
    if (commandDue()) {
      sendCommand(
        VC::VEHICLE_CMD_DO_VTOL_TRANSITION, transition_to_fw_ ? 4.0F : 3.0F);
    }
    if (transition_goal_) {
      auto feedback = std::make_shared<TransitionVtol::Feedback>();
      feedback->phase = transition_to_fw_ ? "transitioning to fixed wing" :
        "transitioning to multicopter";
      feedback->current_vehicle_mode = state.vehicle_mode;
      transition_goal_->publish_feedback(feedback);
    }
  }

  void finishOperation()
  {
    publishInactiveIntent();
    operation_ = Operation::IDLE;
    manual_operation_ = false;
    takeoff_goal_.reset();
    land_goal_.reset();
    transition_goal_.reset();
  }

  void onTimer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_state_) return;
    if (operation_ != Operation::TAKEOFF) processArmOffboard(state_);
    switch (operation_) {
      case Operation::TAKEOFF:
        processTakeoff(state_);
        break;
      case Operation::LAND:
        processLand(state_);
        break;
      case Operation::TRANSITION:
        processTransition(state_);
        break;
      case Operation::IDLE:
        publishInactiveIntent();
        break;
    }
  }

  double takeoff_tolerance_m_ {0.5};
  double landed_altitude_m_ {0.3};
  double command_period_s_ {1.0};
  mutable std::mutex mutex_;
  Operation operation_ {Operation::IDLE};
  VehicleState state_;
  bool have_state_ {false};
  bool arm_offboard_requested_ {false};
  bool transition_to_fw_ {false};
  bool manual_operation_ {false};
  rclcpp::Time last_vehicle_command_ {0, 0, RCL_ROS_TIME};
  std::shared_ptr<TakeoffGoalHandle> takeoff_goal_;
  std::shared_ptr<LandGoalHandle> land_goal_;
  std::shared_ptr<TransitionGoalHandle> transition_goal_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<FlightRequest>::SharedPtr request_sub_;
  rclcpp::Publisher<VC>::SharedPtr command_pub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr intent_pub_;
  rclcpp::Service<Arm>::SharedPtr arm_service_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<Land>::SharedPtr land_server_;
  rclcpp_action::Server<TransitionVtol>::SharedPtr transition_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
