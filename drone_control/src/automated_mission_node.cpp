#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <drone_interfaces/msg/apf_telemetry.hpp>
#include <drone_interfaces/msg/automated_mission_telemetry.hpp>
#include <drone_interfaces/msg/flight_request.hpp>
#include <drone_interfaces/msg/motion_command.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using ApfTelemetry = drone_interfaces::msg::ApfTelemetry;
using AutomatedMissionTelemetry = drone_interfaces::msg::AutomatedMissionTelemetry;
using FlightRequest = drone_interfaces::msg::FlightRequest;
using MotionCommand = drone_interfaces::msg::MotionCommand;
using VehicleState = drone_interfaces::msg::VehicleState;
using std::placeholders::_1;

class AutomatedMissionNode : public rclcpp::Node
{
public:
  AutomatedMissionNode()
  : Node("automated_mission")
  {
    const double goal_x = declare_parameter<double>("goal_x", 700.0);
    const double goal_y = declare_parameter<double>("goal_y", 0.0);
    cruise_altitude_ = std::max(
      2.0, declare_parameter<double>("cruise_altitude", 15.0));
    mission_profile_ = declare_parameter<std::string>("mission_profile", "single");
    const MissionGoal second_goal{
      declare_parameter<double>("goal_2_x", 0.0),
      declare_parameter<double>("goal_2_y", 0.0),
      std::max(2.0, declare_parameter<double>("goal_2_altitude", 15.0))};
    const MissionGoal third_goal{
      declare_parameter<double>("goal_3_x", 750.0),
      declare_parameter<double>("goal_3_y", 15.0),
      std::max(2.0, declare_parameter<double>("goal_3_altitude", 15.0))};
    goals_.push_back({goal_x, goal_y, cruise_altitude_});
    if (mission_profile_ == "three_goal") {
      goals_.push_back(second_goal);
      goals_.push_back(third_goal);
    } else if (mission_profile_ != "single") {
      throw std::invalid_argument("mission_profile must be 'single' or 'three_goal'");
    }
    goal_approach_distance_ = std::max(
      10.0, declare_parameter<double>("goal_approach_distance", 30.0));
    intermediate_goal_tolerance_ = std::max(
      5.0,
      declare_parameter<double>("intermediate_goal_tolerance", 25.0));
    goal_tolerance_ = std::clamp(
      declare_parameter<double>("goal_tolerance", 5.0),
      1.0, goal_approach_distance_ - 1.0);
    landing_handover_altitude_ = std::clamp(
      declare_parameter<double>("landing_handover_altitude", 3.0),
      0.5, cruise_altitude_);
    landing_altitude_tolerance_ = std::max(
      0.1, declare_parameter<double>("landing_altitude_tolerance", 0.5));
    landing_max_horizontal_speed_ = std::max(
      0.1, declare_parameter<double>("landing_max_horizontal_speed", 0.8));
    landing_settle_time_ = std::max(
      0.2, declare_parameter<double>("landing_settle_time", 1.0));
    takeoff_climb_speed_ = std::max(
      0.2, declare_parameter<double>("takeoff_climb_speed", 3.0));
    auto_start_delay_ = std::max(
      1.0, declare_parameter<double>("auto_start_delay", 2.0));
    mc_speed_ = std::max(0.2, declare_parameter<double>("mc_speed", 4.0));
    mc_climb_speed_ = std::max(
      0.2, declare_parameter<double>("mc_climb_speed", 2.0));
    fw_speed_min_ = std::max(
      5.0, declare_parameter<double>("fw_speed_min", 10.0));
    fw_speed_max_ = std::max(
      fw_speed_min_, declare_parameter<double>("fw_speed_max", 20.0));
    fw_speed_cruise_ = std::clamp(
      declare_parameter<double>("fw_speed_cruise", 20.0),
      fw_speed_min_, fw_speed_max_);
    avoidance_enabled_ = declare_parameter<bool>("avoidance_enabled", true);
    obstacle_timeout_s_ = std::max(
      0.1, declare_parameter<double>("obstacle_timeout_s", 0.5));

    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&AutomatedMissionNode::onState, this, _1));
    apf_sub_ = create_subscription<ApfTelemetry>(
      "/apf/telemetry", 10, std::bind(&AutomatedMissionNode::onApf, this, _1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/perception/obstacles", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&AutomatedMissionNode::onObstacles, this, _1));
    intent_pub_ = create_publisher<MotionCommand>("/motion/autonomous_intent", 10);
    flight_request_pub_ = create_publisher<FlightRequest>("/flight/request", 10);
    auto telemetry_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    telemetry_pub_ = create_publisher<AutomatedMissionTelemetry>(
      "/automated_controller/telemetry", telemetry_qos);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&AutomatedMissionNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "Automated mission ready: %zu goal(s), profile=%s, FW=%.1fm/s",
      goals_.size(), mission_profile_.c_str(), fw_speed_cruise_);
    for (std::size_t index = 0; index < goals_.size(); ++index) {
      const auto & goal = goals_[index];
      RCLCPP_INFO(
        get_logger(), "  goal %zu/%zu ENU=(%.1f, %.1f, %.1f)",
        index + 1, goals_.size(), goal.x, goal.y, goal.altitude);
    }
  }

private:
  struct MissionGoal
  {
    double x;
    double y;
    double altitude;
  };

  enum class MissionState
  {
    WAITING_FOR_FCU,
    PRIMING_OFFBOARD,
    TAKEOFF,
    TRANSITION_TO_FW,
    CRUISE_FW,
    TRANSITION_TO_MC,
    APPROACH_MC,
    LAND,
    DONE
  };

  void onState(const VehicleState::SharedPtr message)
  {
    if (!message->position_valid) return;
    if (metrics_tracking_ && have_state_) {
      const double dx = message->position_enu.x - state_.position_enu.x;
      const double dy = message->position_enu.y - state_.position_enu.y;
      const double dz = message->position_enu.z - state_.position_enu.z;
      const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (std::isfinite(step) && step < 5.0) path_length_ += step;
    }
    state_ = *message;
    have_state_ = true;
    updateAttitude();

    if (!metrics_tracking_) return;
    const auto & goal = currentGoal();
    const double route_x = goal.x - leg_start_x_;
    const double route_y = goal.y - leg_start_y_;
    const double route_length = std::hypot(route_x, route_y);
    if (route_length > 1e-3) {
      const double cross_track = std::fabs(
        route_x * (state_.position_enu.y - leg_start_y_) -
        route_y * (state_.position_enu.x - leg_start_x_)) / route_length;
      max_cross_track_error_ = std::max(max_cross_track_error_, cross_track);
    }
  }

  void updateAttitude()
  {
    if (!state_.attitude_valid) return;
    const auto & q = state_.attitude_body_to_ned;
    const double sin_roll = 2.0 * (q.w * q.x + q.y * q.z);
    const double cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll_rad_ = std::atan2(sin_roll, cos_roll);
    const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
    pitch_rad_ = std::asin(std::clamp(sin_pitch, -1.0, 1.0));
  }

  void onApf(const ApfTelemetry::SharedPtr message)
  {
    apf_ = *message;
    have_apf_ = true;
    if (message->avoidance_active && !avoidance_active_) ++avoidance_activations_;
    avoidance_active_ = message->avoidance_active;
    if (message->nearest_path_obstacle_distance_m >= 0.0) {
      minimum_lidar_distance_ = std::min(
        minimum_lidar_distance_, message->nearest_path_obstacle_distance_m);
    }
  }

  void onObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    double nearest = std::numeric_limits<double>::infinity();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!have_state_) continue;
        const double dx = *x - state_.position_enu.x;
        const double dy = *y - state_.position_enu.y;
        const double dz = *z - state_.position_enu.z;
        nearest = std::min(nearest, std::sqrt(dx * dx + dy * dy + dz * dz));
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Invalid obstacle cloud: %s", error.what());
      return;
    }
    nearest_lidar_distance_ = nearest;
    if (std::isfinite(nearest)) {
      minimum_lidar_distance_ = std::min(minimum_lidar_distance_, nearest);
    }
    obstacles_received_ = now();
    have_obstacles_ = true;
  }

  bool obstaclesFresh() const
  {
    return have_obstacles_ && (now() - obstacles_received_).seconds() <= obstacle_timeout_s_;
  }

  double distanceToGoal() const
  {
    const auto & goal = currentGoal();
    return std::hypot(
      goal.x - state_.position_enu.x, goal.y - state_.position_enu.y);
  }

  std::pair<double, double> goalDirection() const
  {
    const auto & goal = currentGoal();
    const double dx = goal.x - state_.position_enu.x;
    const double dy = goal.y - state_.position_enu.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 1e-3) return {0.0, 0.0};
    return {dx / distance, dy / distance};
  }

  void publishIntent(uint8_t mode, double speed, double vertical_speed, bool hold_altitude)
  {
    const auto [direction_x, direction_y] = goalDirection();
    MotionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "map";
    command.vehicle_mode = mode;
    command.source = MotionCommand::SOURCE_AUTONOMOUS;
    command.active = true;
    command.velocity_enu.x = speed * direction_x;
    command.velocity_enu.y = speed * direction_y;
    command.velocity_enu.z = vertical_speed;
    command.hold_altitude = hold_altitude;
    command.target_altitude_m = currentGoal().altitude;
    intent_pub_->publish(command);
  }

  const MissionGoal & currentGoal() const
  {
    return goals_[active_goal_index_];
  }

  bool finalGoalActive() const
  {
    return active_goal_index_ + 1 == goals_.size();
  }

  void initializeRouteMetrics()
  {
    leg_start_x_ = state_.position_enu.x;
    leg_start_y_ = state_.position_enu.y;
    active_goal_index_ = 0;
    completed_route_distance_ = 0.0;
    total_route_distance_ = std::hypot(
      goals_.front().x - leg_start_x_, goals_.front().y - leg_start_y_);
    for (std::size_t index = 1; index < goals_.size(); ++index) {
      total_route_distance_ += std::hypot(
        goals_[index].x - goals_[index - 1].x,
        goals_[index].y - goals_[index - 1].y);
    }
    current_leg_nominal_length_ = std::hypot(
      goals_.front().x - leg_start_x_, goals_.front().y - leg_start_y_);
  }

  void advanceGoal()
  {
    completed_route_distance_ += current_leg_nominal_length_;
    const auto previous_goal = currentGoal();
    ++active_goal_index_;
    leg_start_x_ = previous_goal.x;
    leg_start_y_ = previous_goal.y;
    const auto & next_goal = currentGoal();
    current_leg_nominal_length_ = std::hypot(
      next_goal.x - previous_goal.x, next_goal.y - previous_goal.y);
    RCLCPP_INFO(
      get_logger(), "[goal %zu/%zu reached] continuing in FW to (%.1f, %.1f, %.1f)",
      active_goal_index_, goals_.size(), next_goal.x, next_goal.y, next_goal.altitude);
  }

  double missionProgressDistance() const
  {
    const double leg_progress = std::clamp(
      current_leg_nominal_length_ - distanceToGoal(), 0.0, current_leg_nominal_length_);
    return std::clamp(
      completed_route_distance_ + leg_progress, 0.0, total_route_distance_);
  }

  void publishInactiveIntent()
  {
    MotionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "map";
    command.source = MotionCommand::SOURCE_AUTONOMOUS;
    intent_pub_->publish(command);
  }

  bool requestDue(double period_s)
  {
    if ((now() - last_request_).seconds() < period_s) return false;
    last_request_ = now();
    return true;
  }

  void requestFlight(uint8_t request)
  {
    FlightRequest message;
    message.header.stamp = now();
    message.request = request;
    flight_request_pub_->publish(message);
  }

  std::string stateName() const
  {
    switch (mission_state_) {
      case MissionState::WAITING_FOR_FCU: return "waiting_for_fcu";
      case MissionState::PRIMING_OFFBOARD: return "priming_offboard";
      case MissionState::TAKEOFF: return "takeoff";
      case MissionState::TRANSITION_TO_FW: return "transition_to_fw";
      case MissionState::CRUISE_FW: return "cruise_fw";
      case MissionState::TRANSITION_TO_MC: return "transition_to_mc";
      case MissionState::APPROACH_MC: return "approach_mc";
      case MissionState::LAND: return "land";
      case MissionState::DONE: return "done";
    }
    return "unknown";
  }

  void publishTelemetry()
  {
    AutomatedMissionTelemetry message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    message.state = stateName();
    message.result = mission_result_;
    message.armed = have_state_ && state_.armed;
    message.offboard = have_state_ && state_.offboard;
    message.fixed_wing = have_state_ &&
      state_.vehicle_mode == VehicleState::MODE_FIXED_WING;
    message.avoidance_active = avoidance_active_;
    message.vehicle_state_received = have_state_;
    message.attitude_ready = have_state_ && state_.attitude_valid;
    message.obstacles_ready = !avoidance_enabled_ || obstaclesFresh();
    message.elapsed_time = metrics_tracking_ ? (now() - mission_started_).seconds() : 0.0;
    message.goal_distance = have_state_ ? distanceToGoal() : -1.0;
    const double mission_progress = have_state_ && metrics_tracking_ ?
      missionProgressDistance() : 0.0;
    message.mission_progress_distance = mission_progress;
    message.mission_remaining_distance = metrics_tracking_ ?
      std::max(0.0, total_route_distance_ - mission_progress) : -1.0;
    message.active_goal_index = static_cast<uint32_t>(active_goal_index_);
    message.goal_count = static_cast<uint32_t>(goals_.size());
    message.east = state_.position_enu.x;
    message.north = state_.position_enu.y;
    message.altitude = state_.position_enu.z;
    message.roll_rad = roll_rad_;
    message.pitch_rad = pitch_rad_;
    message.path_length = path_length_;
    message.max_cross_track_error = max_cross_track_error_;
    message.nearest_path_obstacle_distance = have_apf_ ?
      apf_.nearest_path_obstacle_distance_m : -1.0;
    message.nearest_lidar_distance = std::isfinite(nearest_lidar_distance_) ?
      nearest_lidar_distance_ : -1.0;
    message.minimum_lidar_distance = std::isfinite(minimum_lidar_distance_) ?
      minimum_lidar_distance_ : -1.0;
    message.avoidance_activations = avoidance_activations_;
    if (have_apf_) {
      message.attractive_force_enu = apf_.attractive_force_enu;
      message.repulsive_force_enu = apf_.repulsive_force_enu;
      message.safe_command_enu = apf_.safe_command_enu;
    }
    telemetry_pub_->publish(message);
  }

  void onTimer()
  {
    const auto stamp = now();
    switch (mission_state_) {
      case MissionState::WAITING_FOR_FCU:
        publishInactiveIntent();
        if (have_state_ && state_.attitude_valid &&
          (!avoidance_enabled_ || obstaclesFresh()))
        {
          state_started_ = stamp;
          mission_state_ = MissionState::PRIMING_OFFBOARD;
          RCLCPP_INFO(get_logger(), "[FCU ready] priming Offboard setpoints");
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "[startup waiting] vehicle_state=%s attitude=%s obstacles=%s",
            have_state_ ? "ready" : "missing",
            have_state_ && state_.attitude_valid ? "ready" : "missing",
            !avoidance_enabled_ || obstaclesFresh() ? "ready" : "missing");
        }
        break;

      case MissionState::PRIMING_OFFBOARD:
        publishIntent(MotionCommand::MODE_MULTICOPTER, 0.0, 0.0, false);
        if ((stamp - state_started_).seconds() >= auto_start_delay_ && requestDue(1.0)) {
          requestFlight(FlightRequest::ARM_OFFBOARD);
        }
        if (!state_.armed || !state_.offboard) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "[startup waiting] armed=%s offboard=%s",
            state_.armed ? "ready" : "missing",
            state_.offboard ? "ready" : "missing");
        }
        if (state_.armed && state_.offboard) {
          initializeRouteMetrics();
          mission_started_ = stamp;
          metrics_tracking_ = true;
          mission_result_ = "running";
          mission_state_ = MissionState::TAKEOFF;
          RCLCPP_INFO(
            get_logger(), "[armed + offboard] MC takeoff to %.1fm",
            currentGoal().altitude);
        }
        break;

      case MissionState::TAKEOFF: {
        const double climb = std::clamp(
          0.8 * (currentGoal().altitude - state_.position_enu.z),
          0.0, takeoff_climb_speed_);
        publishIntent(MotionCommand::MODE_MULTICOPTER, 0.0, climb, true);
        if (state_.position_enu.z >= currentGoal().altitude - 1.0) {
          requestFlight(FlightRequest::TRANSITION_TO_FW);
          last_request_ = stamp;
          mission_state_ = MissionState::TRANSITION_TO_FW;
          RCLCPP_INFO(
            get_logger(), "[altitude %.1fm] transition to FW", state_.position_enu.z);
        }
        break;
      }

      case MissionState::TRANSITION_TO_FW:
        publishIntent(MotionCommand::MODE_FIXED_WING, fw_speed_cruise_, 0.0, true);
        if (state_.vehicle_mode == VehicleState::MODE_FIXED_WING) {
          mission_state_ = MissionState::CRUISE_FW;
          RCLCPP_INFO(get_logger(), "[FW] autonomous cruise to goal");
        } else if (requestDue(2.0)) {
          requestFlight(FlightRequest::TRANSITION_TO_FW);
        }
        break;

      case MissionState::CRUISE_FW:
        publishIntent(MotionCommand::MODE_FIXED_WING, fw_speed_cruise_, 0.0, true);
        if (!finalGoalActive() && distanceToGoal() <= intermediate_goal_tolerance_) {
          advanceGoal();
        } else if (finalGoalActive() && distanceToGoal() <= goal_approach_distance_) {
          requestFlight(FlightRequest::TRANSITION_TO_MC);
          last_request_ = stamp;
          mission_state_ = MissionState::TRANSITION_TO_MC;
          RCLCPP_INFO(
            get_logger(), "[goal %.1fm] transition to MC for approach", distanceToGoal());
        }
        break;

      case MissionState::TRANSITION_TO_MC:
        if (state_.vehicle_mode == VehicleState::MODE_FIXED_WING) {
          publishIntent(MotionCommand::MODE_FIXED_WING, fw_speed_cruise_, 0.0, true);
          if (requestDue(1.0)) requestFlight(FlightRequest::TRANSITION_TO_MC);
        } else {
          publishIntent(MotionCommand::MODE_MULTICOPTER, 0.0, 0.0, false);
          landing_ready_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          mission_state_ = MissionState::APPROACH_MC;
          RCLCPP_INFO(get_logger(), "[MC] final approach");
        }
        break;

      case MissionState::APPROACH_MC: {
        const double vertical = std::clamp(
          0.8 * (landing_handover_altitude_ - state_.position_enu.z),
          -mc_climb_speed_, mc_climb_speed_);
        const double approach_speed = std::min(mc_speed_, 0.5 * distanceToGoal());
        publishIntent(MotionCommand::MODE_MULTICOPTER, approach_speed, vertical, false);
        const double horizontal_speed = std::hypot(
          state_.velocity_enu.x, state_.velocity_enu.y);
        const bool landing_ready =
          distanceToGoal() <= goal_tolerance_ &&
          state_.position_enu.z <=
          landing_handover_altitude_ + landing_altitude_tolerance_ &&
          horizontal_speed <= landing_max_horizontal_speed_ && !avoidance_active_;
        if (!landing_ready) {
          landing_ready_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          break;
        }
        if (landing_ready_since_.nanoseconds() == 0) landing_ready_since_ = stamp;
        if ((stamp - landing_ready_since_).seconds() >= landing_settle_time_) {
          requestFlight(FlightRequest::LAND);
          last_request_ = stamp;
          mission_state_ = MissionState::LAND;
          RCLCPP_INFO(
            get_logger(),
            "[landing handover] goal=%.1fm altitude=%.1fm speed=%.2fm/s",
            distanceToGoal(), state_.position_enu.z, horizontal_speed);
        }
        break;
      }

      case MissionState::LAND:
        publishInactiveIntent();
        if (!state_.armed) {
          const double final_distance = distanceToGoal();
          mission_result_ = final_distance <= goal_tolerance_ ? "success" : "landing_miss";
          mission_state_ = MissionState::DONE;
          RCLCPP_INFO(
            get_logger(), "[landed %.1fm from goal] result=%s",
            final_distance, mission_result_.c_str());
        } else if (requestDue(2.0)) {
          requestFlight(FlightRequest::LAND);
        }
        break;

      case MissionState::DONE:
        publishInactiveIntent();
        break;
    }
    publishTelemetry();
  }

  std::vector<MissionGoal> goals_;
  std::string mission_profile_ {"single"};
  std::size_t active_goal_index_ {0};
  double cruise_altitude_ {15.0};
  double goal_approach_distance_ {30.0};
  double intermediate_goal_tolerance_ {25.0};
  double goal_tolerance_ {5.0};
  double landing_handover_altitude_ {3.0};
  double landing_altitude_tolerance_ {0.5};
  double landing_max_horizontal_speed_ {0.8};
  double landing_settle_time_ {1.0};
  double takeoff_climb_speed_ {3.0};
  double auto_start_delay_ {2.0};
  double mc_speed_ {4.0};
  double mc_climb_speed_ {2.0};
  double fw_speed_min_ {10.0};
  double fw_speed_max_ {20.0};
  double fw_speed_cruise_ {20.0};
  double obstacle_timeout_s_ {0.5};
  bool avoidance_enabled_ {true};
  MissionState mission_state_ {MissionState::WAITING_FOR_FCU};
  VehicleState state_;
  ApfTelemetry apf_;
  bool have_state_ {false};
  bool have_apf_ {false};
  bool have_obstacles_ {false};
  bool avoidance_active_ {false};
  bool metrics_tracking_ {false};
  std::string mission_result_ {"pending"};
  double leg_start_x_ {0.0};
  double leg_start_y_ {0.0};
  double completed_route_distance_ {0.0};
  double current_leg_nominal_length_ {0.0};
  double total_route_distance_ {0.0};
  double path_length_ {0.0};
  double max_cross_track_error_ {0.0};
  double nearest_lidar_distance_ {std::numeric_limits<double>::infinity()};
  double minimum_lidar_distance_ {std::numeric_limits<double>::infinity()};
  double roll_rad_ {0.0};
  double pitch_rad_ {0.0};
  uint32_t avoidance_activations_ {0};
  rclcpp::Time state_started_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_request_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacles_received_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time landing_ready_since_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time mission_started_ {0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<ApfTelemetry>::SharedPtr apf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<MotionCommand>::SharedPtr intent_pub_;
  rclcpp::Publisher<FlightRequest>::SharedPtr flight_request_pub_;
  rclcpp::Publisher<AutomatedMissionTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutomatedMissionNode>());
  rclcpp::shutdown();
  return 0;
}
