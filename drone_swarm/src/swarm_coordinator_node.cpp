#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <drone_interfaces/msg/route_target.hpp>
#include <drone_interfaces/msg/swarm_assignment.hpp>
#include <drone_interfaces/msg/swarm_drone_heartbeat.hpp>
#include <drone_interfaces/msg/swarm_drone_state.hpp>
#include <drone_interfaces/msg/swarm_mission_command.hpp>
#include <drone_interfaces/msg/swarm_mission_feedback.hpp>
#include <drone_interfaces/msg/swarm_route.hpp>
#include <drone_interfaces/msg/swarm_state.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <drone_interfaces/srv/add_swarm_target.hpp>
#include <drone_interfaces/srv/register_swarm_drone.hpp>
#include <drone_interfaces/srv/remove_swarm_target.hpp>
#include <drone_interfaces/srv/swarm_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drone_swarm/flight_cost_model.hpp"
#include "drone_swarm/route_solver.hpp"

using AddSwarmTarget = drone_interfaces::srv::AddSwarmTarget;
using RouteTarget = drone_interfaces::msg::RouteTarget;
using RegisterSwarmDrone = drone_interfaces::srv::RegisterSwarmDrone;
using RemoveSwarmTarget = drone_interfaces::srv::RemoveSwarmTarget;
using SwarmAssignment = drone_interfaces::msg::SwarmAssignment;
using SwarmCommand = drone_interfaces::srv::SwarmCommand;
using SwarmDroneHeartbeat = drone_interfaces::msg::SwarmDroneHeartbeat;
using SwarmDroneState = drone_interfaces::msg::SwarmDroneState;
using SwarmMissionCommand = drone_interfaces::msg::SwarmMissionCommand;
using SwarmMissionFeedback = drone_interfaces::msg::SwarmMissionFeedback;
using SwarmRoute = drone_interfaces::msg::SwarmRoute;
using SwarmState = drone_interfaces::msg::SwarmState;
using VehicleState = drone_interfaces::msg::VehicleState;
using std::placeholders::_1;
using std::placeholders::_2;

namespace drone_swarm
{

class SwarmCoordinatorNode : public rclcpp::Node
{
public:
  SwarmCoordinatorNode()
  : Node("swarm_coordinator")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    drone_state_timeout_s_ = std::max(
      0.1, declare_parameter<double>("drone_state_timeout_sec", 1.0));
    feedback_timeout_s_ = std::max(
      0.1, declare_parameter<double>("feedback_timeout_sec", 5.0));
    min_altitude_m_ = declare_parameter<double>("geofence.min_altitude_m", 2.0);
    geofence_cube_size_m_ = std::max(
      1.0, declare_parameter<double>("geofence.cube_size_m", 2000.0));
    no_fly_zone_values_ = declare_parameter<std::vector<double>>(
      "geofence.no_fly_zones", std::vector<double>{});
    discover_drones_ = declare_parameter<bool>("discover_drones", true);
    require_registration_ = declare_parameter<bool>("require_registration", true);
    heartbeat_period_s_ = std::max(
      0.1, declare_parameter<double>("heartbeat_period_sec", 0.5));
    lease_timeout_s_ = std::max(
      2.0 * heartbeat_period_s_, declare_parameter<double>("lease_timeout_sec", 3.0));
    route_improvement_passes_ = static_cast<std::size_t>(std::max<int64_t>(
      0, declare_parameter<int64_t>("route_improvement_passes", 50)));
    home_altitude_above_origin_m_ = std::max(
      min_altitude_m_, declare_parameter<double>("home.altitude_above_origin_m", 15.0));
    home_cruise_speed_m_s_ = std::max(
      0.1, declare_parameter<double>("home.cruise_speed_m_s", 15.0));
    home_precision_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("home.precision_speed_m_s", 3.0));
    home_use_fixed_wing_ = declare_parameter<bool>("home.use_fixed_wing", true);
    min_drone_speed_m_s_ = std::max(
      0.1, declare_parameter<double>("speed_override.min_m_s", 10.0));
    max_drone_speed_m_s_ = std::max(
      min_drone_speed_m_s_, declare_parameter<double>("speed_override.max_m_s", 20.0));
    min_lidar_range_m_ = std::max(
      1.0, declare_parameter<double>("lidar_range.min_m", 70.0));
    max_lidar_range_m_ = std::max(
      min_lidar_range_m_, declare_parameter<double>("lidar_range.max_m", 300.0));
    default_lidar_range_m_ = std::clamp(
      declare_parameter<double>("lidar_range.default_m", 70.0),
      min_lidar_range_m_, max_lidar_range_m_);
    battery_low_pct_ = std::clamp(
      declare_parameter<double>("battery.low_pct", 40.0), 1.0, 100.0);
    battery_critical_pct_ = std::clamp(
      declare_parameter<double>("battery.critical_pct", 25.0), 1.0, battery_low_pct_);
    battery_penalty_max_s_ = std::max(
      0.0, declare_parameter<double>("battery.low_penalty_max_sec", 120.0));
    battery_reserve_s_ = std::max(
      0.0, declare_parameter<double>("battery.flight_time_reserve_sec", 60.0));
    active_route_penalty_s_ = std::max(
      0.0, declare_parameter<double>("routing.active_route_change_penalty_sec", 30.0));
    routing_vertical_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("routing.vertical_speed_m_s", 3.0));
    battery_energy_reserve_pct_ = std::clamp(
      declare_parameter<double>("battery.energy_reserve_pct", 15.0), 0.0, 90.0);
    FlightCostParameters flight_cost_parameters;
    flight_cost_parameters.vertical_speed_m_s = routing_vertical_speed_m_s_;
    flight_cost_parameters.fixed_wing_turn_rate_rad_s = std::max(
      1.0, declare_parameter<double>("routing.fixed_wing_turn_rate_deg_s", 25.0)) *
      3.14159265358979323846 / 180.0;
    flight_cost_parameters.fixed_wing_detour_factor = std::max(
      1.0, declare_parameter<double>("routing.fixed_wing_detour_factor", 1.08));
    flight_cost_parameters.multicopter_detour_factor = std::max(
      1.0, declare_parameter<double>("routing.multicopter_detour_factor", 1.03));
    flight_cost_parameters.transition_time_s = std::max(
      0.0, declare_parameter<double>("routing.vtol_transition_time_sec", 8.0));
    flight_cost_parameters.multicopter_power_w = std::max(
      0.0, declare_parameter<double>("routing.multicopter_power_w", 450.0));
    flight_cost_parameters.fixed_wing_power_w = std::max(
      0.0, declare_parameter<double>("routing.fixed_wing_power_w", 220.0));
    flight_cost_parameters.transition_power_w = std::max(
      0.0, declare_parameter<double>("routing.vtol_transition_power_w", 650.0));
    flight_cost_parameters.climb_power_per_m_s_w = std::max(
      0.0, declare_parameter<double>("routing.climb_power_per_m_s_w", 100.0));
    flight_cost_parameters.multicopter_speed_power_coefficient = std::max(
      0.0, declare_parameter<double>("routing.multicopter_speed_power_coefficient", 2.0));
    flight_cost_parameters.fixed_wing_reference_speed_m_s = std::max(
      1.0, declare_parameter<double>("routing.fixed_wing_reference_speed_m_s", 15.0));
    flight_cost_model_ = std::make_unique<FlightCostModel>(flight_cost_parameters);
    if (no_fly_zone_values_.size() % 6 != 0) {
      throw std::runtime_error("geofence.no_fly_zones must contain groups of 6 values");
    }

    configureDrones();

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    const auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    mission_command_pub_ = create_publisher<SwarmMissionCommand>(
      "/swarm/mission_command", command_qos);
    mission_feedback_sub_ = create_subscription<SwarmMissionFeedback>(
      "/swarm/mission_feedback", feedback_qos,
      std::bind(&SwarmCoordinatorNode::onMissionFeedback, this, _1));
    heartbeat_sub_ = create_subscription<SwarmDroneHeartbeat>(
      "/swarm/drone_heartbeat", rclcpp::QoS(100).reliable(),
      std::bind(&SwarmCoordinatorNode::onDroneHeartbeat, this, _1));
    add_target_service_ = create_service<AddSwarmTarget>(
      "/swarm/add_target", std::bind(&SwarmCoordinatorNode::addTarget, this, _1, _2));
    remove_target_service_ = create_service<RemoveSwarmTarget>(
      "/swarm/remove_target", std::bind(&SwarmCoordinatorNode::removeTarget, this, _1, _2));
    command_service_ = create_service<SwarmCommand>(
      "/swarm/command", std::bind(&SwarmCoordinatorNode::handleCommand, this, _1, _2));
    register_drone_service_ = create_service<RegisterSwarmDrone>(
      "/swarm/register_drone",
      std::bind(&SwarmCoordinatorNode::registerDrone, this, _1, _2));
    state_pub_ = create_publisher<SwarmState>(
      "/swarm/state", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SwarmCoordinatorNode::onTimer, this));
    last_discovery_ = std::chrono::steady_clock::now() - std::chrono::seconds(2);

    RCLCPP_INFO(
      get_logger(),
      "Coordinator ready with %zu configured drone(s); broadcast command and feedback enabled",
      drones_.size());
  }

private:
  struct TargetRecord
  {
    uint64_t id {0};
    geometry_msgs::msg::PoseStamped target;
    double cruise_speed_m_s {0.0};
    bool use_fixed_wing {true};
    bool preserve_on_cancel {true};
  };

  struct DroneRecord
  {
    std::string id;
    std::string drone_namespace;
    std::string state_topic;
    VehicleState state;
    geometry_msgs::msg::Point geofence_origin;
    bool have_state {false};
    bool have_geofence_origin {false};
    bool busy {false};
    bool registered {false};
    bool px4_ready {false};
    bool localized {false};
    bool navigation_ready {false};
    bool lidar_ready {false};
    bool operator_enabled {true};
    bool has_lidar {false};
    bool supports_fixed_wing {false};
    bool supports_vtol {false};
    bool has_speed_override {false};
    double speed_override_m_s {0.0};
    double lidar_range_m {70.0};
    bool battery_valid {false};
    double battery_remaining_pct {0.0};
    double battery_time_remaining_s {-1.0};
    double battery_power_w {0.0};
    double battery_capacity_wh {0.0};
    double battery_remaining_energy_wh {0.0};
    uint8_t battery_state {SwarmDroneHeartbeat::BATTERY_STATE_UNKNOWN};
    bool battery_available_for_tasks {true};
    bool safety_excluded {false};
    bool return_home_active {false};
    uint8_t last_safety_event_state {SwarmDroneHeartbeat::BATTERY_STATE_UNKNOWN};
    std::string boot_id;
    std::string navigate_action;
    std::string arm_service;
    std::string takeoff_action;
    rclcpp::Time last_update;
    std::chrono::steady_clock::time_point last_heartbeat;
    rclcpp::Subscription<VehicleState>::SharedPtr state_sub;
  };

  struct RouteRecord
  {
    uint64_t token {0};
    uint64_t route_id {0};
    std::string drone_id;
    uint8_t purpose {SwarmRoute::PURPOSE_MISSION};
    std::vector<TargetRecord> targets;
    std::vector<double> leg_costs;
    double total_cost {0.0};
    bool accepted {false};
    bool finished {false};
    uint8_t state {SwarmAssignment::WAITING};
    uint64_t active_target_id {0};
    uint32_t active_target_index {0};
    uint32_t completed_target_count {0};
    double distance_remaining_m {std::numeric_limits<double>::quiet_NaN()};
    std::string message {"waiting for broadcast"};
  };

  struct AuxiliaryRoute
  {
    uint64_t command_id {0};
    uint64_t mission_id {0};
    uint32_t revision {1};
    RouteRecord route;
  };

  struct SafetyEvent
  {
    std::string drone_id;
    uint8_t battery_state {SwarmDroneHeartbeat::BATTERY_STATE_UNKNOWN};
  };

  enum class DispatchPhase
  {
    IDLE,
    WAITING_FOR_FEEDBACK,
  };

  static std::string normalizeNamespace(std::string drone_namespace)
  {
    if (drone_namespace.empty() || drone_namespace == "/") return "";
    if (drone_namespace.front() != '/') drone_namespace.insert(drone_namespace.begin(), '/');
    while (drone_namespace.size() > 1 && drone_namespace.back() == '/') {
      drone_namespace.pop_back();
    }
    return drone_namespace;
  }

  void configureDrones()
  {
    const auto drone_ids = declare_parameter<std::vector<std::string>>(
      "drone_ids", std::vector<std::string>{"drone_1"});
    const auto drone_namespaces = declare_parameter<std::vector<std::string>>(
      "drone_namespaces", std::vector<std::string>{});
    const auto state_topics = declare_parameter<std::vector<std::string>>(
      "state_topics", std::vector<std::string>{});

    if (!drone_namespaces.empty() && drone_namespaces.size() != drone_ids.size()) {
      throw std::runtime_error("drone_namespaces must be empty or match drone_ids");
    }
    if (!state_topics.empty() && state_topics.size() != drone_ids.size()) {
      throw std::runtime_error("state_topics must be empty or match drone_ids");
    }

    std::set<std::string> unique_ids;
    for (std::size_t index = 0; index < drone_ids.size(); ++index) {
      if (drone_ids[index].empty() || !unique_ids.insert(drone_ids[index]).second) {
        throw std::runtime_error("drone_ids must be non-empty and unique");
      }
      const auto drone_namespace = normalizeNamespace(
        drone_namespaces.empty() ? drone_ids[index] : drone_namespaces[index]);
      addDrone(
        drone_ids[index], drone_namespace,
        state_topics.empty() ? drone_namespace + "/vehicle/state" : state_topics[index]);
    }
  }

  void addDrone(
    const std::string & drone_id,
    const std::string & drone_namespace,
    const std::string & state_topic)
  {
    const auto existing = std::find_if(
      drones_.begin(), drones_.end(),
      [&drone_id](const auto & drone) {return drone->id == drone_id;});
    if (existing != drones_.end()) return;

    auto drone = std::make_shared<DroneRecord>();
    drone->id = drone_id;
    drone->drone_namespace = drone_namespace;
    drone->state_topic = state_topic;
    drone->last_update = now();
    drone->lidar_range_m = default_lidar_range_m_;
    drone->last_heartbeat = std::chrono::steady_clock::now() - std::chrono::hours(1);
    drone->state_sub = create_subscription<VehicleState>(
      drone->state_topic, 10,
      [this, drone](const VehicleState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        drone->state = *message;
        drone->last_update = now();
        drone->have_state = true;
        if (!drone->have_geofence_origin && message->position_valid) {
          drone->geofence_origin = message->position_enu;
          drone->have_geofence_origin = true;
        }
      });
    RCLCPP_INFO(
      get_logger(), "Tracking drone %s: state=%s",
      drone_id.c_str(), state_topic.c_str());
    drones_.push_back(std::move(drone));
  }

  static bool validDroneId(const std::string & drone_id)
  {
    if (drone_id.empty()) return false;
    return std::all_of(
      drone_id.begin(), drone_id.end(),
      [](unsigned char character) {
        return std::isalnum(character) || character == '_' || character == '-';
      });
  }

  bool heartbeatFreshLocked(
    const DroneRecord & drone,
    const std::chrono::steady_clock::time_point & current_time) const
  {
    return drone.registered &&
           std::chrono::duration<double>(current_time - drone.last_heartbeat).count() <=
           lease_timeout_s_;
  }

  void registerDrone(
    const RegisterSwarmDrone::Request::SharedPtr request,
    RegisterSwarmDrone::Response::SharedPtr response)
  {
    response->heartbeat_period_sec = heartbeat_period_s_;
    response->lease_timeout_sec = lease_timeout_s_;
    if (!validDroneId(request->drone_id)) {
      response->message = "drone_id must contain only letters, numbers, '_' or '-'";
      return;
    }
    if (request->boot_id.empty()) {
      response->message = "boot_id must not be empty";
      return;
    }

    const auto drone_namespace = normalizeNamespace(request->drone_namespace);
    if (drone_namespace.empty()) {
      response->message = "drone_namespace must not be empty";
      return;
    }
    const auto expected_state_topic = drone_namespace + "/vehicle/state";
    const auto state_topic = request->state_topic.empty() ?
      expected_state_topic : request->state_topic;

    auto existing = std::find_if(
      drones_.begin(), drones_.end(),
      [&request](const auto & drone) {return drone->id == request->drone_id;});
    if (existing == drones_.end()) {
      addDrone(request->drone_id, drone_namespace, state_topic);
      existing = std::prev(drones_.end());
    }

    const auto current_time = std::chrono::steady_clock::now();
    bool new_session = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto & drone = **existing;
      new_session = !drone.registered || drone.boot_id != request->boot_id;
      const bool same_boot = drone.boot_id.empty() || drone.boot_id == request->boot_id;
      const bool old_lease_active = heartbeatFreshLocked(drone, current_time);
      const bool owns_route = current_routes_.count(drone.id) != 0 ||
        provisional_routes_.count(drone.id) != 0;
      if (!same_boot && (old_lease_active || owns_route || drone.busy)) {
        response->message = "drone_id is already owned by another active boot session";
        return;
      }
      if (drone.state_topic != state_topic && drone.have_state) {
        response->message = "registered drone_id requested a different state topic";
        return;
      }

      drone.registered = true;
      drone.boot_id = request->boot_id;
      drone.drone_namespace = drone_namespace;
      drone.navigate_action = request->navigate_action.empty() ?
        drone_namespace + "/navigate_to" : request->navigate_action;
      drone.arm_service = request->arm_service.empty() ?
        drone_namespace + "/flight/arm" : request->arm_service;
      drone.takeoff_action = request->takeoff_action.empty() ?
        drone_namespace + "/takeoff" : request->takeoff_action;
      drone.has_lidar = request->has_lidar;
      drone.supports_fixed_wing = request->supports_fixed_wing;
      drone.supports_vtol = request->supports_vtol;
      drone.last_heartbeat = current_time;
    }

    response->accepted = true;
    response->message = request->drone_id + " registered in namespace " + drone_namespace;
    if (new_session) {
      RCLCPP_INFO(
        get_logger(), "Accepted swarm registration: id=%s namespace=%s boot=%s",
        request->drone_id.c_str(), drone_namespace.c_str(), request->boot_id.c_str());
    }
  }

  void onDroneHeartbeat(const SwarmDroneHeartbeat::SharedPtr heartbeat)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::find_if(
      drones_.begin(), drones_.end(),
      [&heartbeat](const auto & drone) {return drone->id == heartbeat->drone_id;});
    if (found == drones_.end() || !(*found)->registered ||
      (*found)->boot_id != heartbeat->boot_id)
    {
      return;
    }
    auto & drone = **found;
    drone.last_heartbeat = std::chrono::steady_clock::now();
    drone.px4_ready = heartbeat->px4_ready;
    drone.localized = heartbeat->localized;
    drone.navigation_ready = heartbeat->navigation_ready;
    drone.lidar_ready = heartbeat->lidar_ready;
    drone.battery_valid = heartbeat->battery_valid;
    drone.battery_remaining_pct = heartbeat->battery_remaining_pct;
    drone.battery_time_remaining_s = heartbeat->battery_time_remaining_s;
    drone.battery_power_w = heartbeat->battery_power_w;
    drone.battery_capacity_wh = heartbeat->battery_capacity_wh;
    drone.battery_remaining_energy_wh = heartbeat->battery_remaining_energy_wh;
    drone.battery_state = heartbeat->battery_state;
    drone.battery_available_for_tasks = heartbeat->available_for_tasks;
    const bool safety_request = heartbeat->return_home_requested ||
      heartbeat->emergency_land_requested;
    if (safety_request && drone.last_safety_event_state != heartbeat->battery_state) {
      drone.safety_excluded = true;
      drone.return_home_active = heartbeat->return_home_requested;
      drone.last_safety_event_state = heartbeat->battery_state;
      const SafetyEvent event{drone.id, heartbeat->battery_state};
      if (heartbeat->emergency_land_requested) {
        pending_safety_events_.erase(
          std::remove_if(
            pending_safety_events_.begin(), pending_safety_events_.end(),
            [&drone](const SafetyEvent & pending) {return pending.drone_id == drone.id;}),
          pending_safety_events_.end());
        pending_safety_events_.insert(pending_safety_events_.begin(), event);
      } else {
        pending_safety_events_.push_back(event);
      }
      status_message_ = drone.id + " reported a battery safety event";
    } else if (!safety_request) {
      drone.last_safety_event_state = SwarmDroneHeartbeat::BATTERY_STATE_UNKNOWN;
    }
  }

  void discoverDrones()
  {
    if (!discover_drones_) return;
    const auto current_time = std::chrono::steady_clock::now();
    if (current_time - last_discovery_ < std::chrono::seconds(1)) return;
    last_discovery_ = current_time;

    constexpr const char * suffix = "/vehicle/state";
    const std::string expected_type = "drone_interfaces/msg/VehicleState";
    for (const auto & topic : get_topic_names_and_types()) {
      const auto suffix_length = std::char_traits<char>::length(suffix);
      if (topic.first.size() <= suffix_length ||
        topic.first.compare(topic.first.size() - suffix_length, suffix_length, suffix) != 0 ||
        std::find(topic.second.begin(), topic.second.end(), expected_type) == topic.second.end())
      {
        continue;
      }
      const auto drone_namespace = topic.first.substr(0, topic.first.size() - suffix_length);
      const auto separator = drone_namespace.find_last_of('/');
      const auto drone_id = drone_namespace.substr(separator + 1);
      if (!drone_id.empty()) {
        addDrone(drone_id, drone_namespace, topic.first);
        if (!require_registration_) findDrone(drone_id)->registered = true;
      }
    }
  }

  std::pair<bool, std::string> validateGeneralTarget(
    const geometry_msgs::msg::PoseStamped & target) const
  {
    if (!target.header.frame_id.empty() && target.header.frame_id != map_frame_) {
      return {false, "target frame must be " + map_frame_};
    }
    const auto & point = target.pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return {false, "target contains a non-finite coordinate"};
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
    return {true, "target is valid"};
  }

  bool targetValidForDrone(const TargetRecord & target, const DroneRecord & drone) const
  {
    if (!drone.have_geofence_origin) return false;
    const auto & point = target.target.pose.position;
    const double half_size = 0.5 * geofence_cube_size_m_;
    return std::fabs(point.x - drone.geofence_origin.x) <= half_size &&
           std::fabs(point.y - drone.geofence_origin.y) <= half_size &&
           std::fabs(point.z - drone.geofence_origin.z) <= half_size;
  }

  void addTarget(
    const AddSwarmTarget::Request::SharedPtr request,
    AddSwarmTarget::Response::SharedPtr response)
  {
    const auto validation = validateGeneralTarget(request->target);
    if (!validation.first) {
      response->message = validation.second;
      RCLCPP_WARN(get_logger(), "Rejected pending target: %s", response->message.c_str());
      return;
    }

    TargetRecord target;
    target.target = request->target;
    target.target.header.frame_id = map_frame_;
    target.cruise_speed_m_s = request->cruise_speed_m_s;
    target.use_fixed_wing = request->use_fixed_wing;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target.id = next_target_id_++;
      pending_targets_.push_back(target);
      invalidatePlannedRoutesLocked();
      response->pending_target_count = pending_targets_.size();
      status_message_ = "target " + std::to_string(target.id) + " added to pending buffer";
    }
    response->accepted = true;
    response->target_id = target.id;
    response->message = status_message_;
    RCLCPP_INFO(
      get_logger(), "Buffered target %lu at (%.1f, %.1f, %.1f); no routing started",
      target.id, target.target.pose.position.x, target.target.pose.position.y,
      target.target.pose.position.z);
  }

  void handleCommand(
    const SwarmCommand::Request::SharedPtr request,
    SwarmCommand::Response::SharedPtr response)
  {
    switch (request->command) {
      case SwarmCommand::Request::CALCULATE:
        response->accepted = calculateRoutes(false, response->message);
        break;
      case SwarmCommand::Request::RECALCULATE:
        response->accepted = calculateRoutes(true, response->message);
        break;
      case SwarmCommand::Request::START_MISSION:
        response->accepted = dispatchPlannedMission(response->message);
        break;
      case SwarmCommand::Request::CLEAR_PENDING_TARGETS:
        response->accepted = clearPendingTargets(response->message);
        break;
      case SwarmCommand::Request::CANCEL_ACTIVE_MISSION:
        response->accepted = cancelActiveMission(response->message);
        break;
      case SwarmCommand::Request::RETURN_HOME:
        response->accepted = startHomeMission(request->drone_id, response->message);
        break;
      case SwarmCommand::Request::SET_DRONE_SPEED:
        response->accepted = setDroneSpeed(
          request->drone_id, request->cruise_speed_m_s, false, response->message);
        break;
      case SwarmCommand::Request::CLEAR_DRONE_SPEED:
        response->accepted = setDroneSpeed(
          request->drone_id, 0.0, true, response->message);
        break;
      case SwarmCommand::Request::LAND:
        response->accepted = landDrones(request->drone_id, response->message);
        break;
      case SwarmCommand::Request::DISABLE_DRONE:
        response->accepted = setDroneEnabled(request->drone_id, false, response->message);
        break;
      case SwarmCommand::Request::ENABLE_DRONE:
        response->accepted = setDroneEnabled(request->drone_id, true, response->message);
        break;
      case SwarmCommand::Request::SET_DRONE_LIDAR_RANGE:
        response->accepted = setDroneLidarRange(
          request->drone_id, request->lidar_range_m, response->message);
        break;
      case SwarmCommand::Request::ARM:
        response->accepted = sendFlightControlCommand(
          SwarmMissionCommand::ARM, request->drone_id, 0.0, 0.0, response->message);
        break;
      case SwarmCommand::Request::TAKEOFF:
        response->accepted = sendFlightControlCommand(
          SwarmMissionCommand::TAKEOFF, request->drone_id,
          request->takeoff_altitude_m, request->takeoff_climb_speed_m_s,
          response->message);
        break;
      default:
        response->message = "unknown swarm command";
        break;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    response->pending_target_count = pending_targets_.size();
    response->active_target_count = active_targets_.size();
    response->assigned_drone_count = planned_routes_.empty() ?
      current_routes_.size() : planned_routes_.size();
  }

  bool setDroneSpeed(
    const std::string & drone_id,
    double speed_m_s,
    bool clear_override,
    std::string & message)
  {
    if (drone_id.empty()) {
      message = "drone_id is required for a speed override";
      return false;
    }
    if (!clear_override && (!std::isfinite(speed_m_s) ||
      speed_m_s < min_drone_speed_m_s_ || speed_m_s > max_drone_speed_m_s_))
    {
      std::ostringstream reason;
      reason << "speed must be between " << min_drone_speed_m_s_ << " and "
             << max_drone_speed_m_s_ << " m/s";
      message = reason.str();
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = std::find_if(
        drones_.begin(), drones_.end(),
        [&drone_id](const auto & drone) {return drone->id == drone_id;});
      if (found == drones_.end()) {
        message = "unknown drone: " + drone_id;
        return false;
      }
      (*found)->has_speed_override = !clear_override;
      (*found)->speed_override_m_s = clear_override ? 0.0 : speed_m_s;
      invalidatePlannedRoutesLocked();
      if (clear_override) {
        message = "cleared speed override for " + drone_id;
      } else {
        std::ostringstream confirmation;
        confirmation << "set " << drone_id << " speed override to " << speed_m_s << " m/s";
        message = confirmation.str();
      }
      status_message_ = message;
    }

    publishSpeedCommand(drone_id, speed_m_s, clear_override);
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  bool setDroneEnabled(
    const std::string & drone_id,
    bool enabled,
    std::string & message)
  {
    if (drone_id.empty()) {
      message = "drone_id is required";
      return false;
    }

    bool canceled_route = false;
    uint64_t mission_id = 0;
    uint32_t revision = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (dispatch_phase_ != DispatchPhase::IDLE) {
        message = "cannot change swarm membership while route feedback is pending";
        return false;
      }
      const auto found = std::find_if(
        drones_.begin(), drones_.end(),
        [&drone_id](const auto & drone) {return drone->id == drone_id;});
      if (found == drones_.end()) {
        message = "unknown drone: " + drone_id;
        return false;
      }
      if (enabled && (*found)->safety_excluded && !(*found)->battery_available_for_tasks) {
        message = drone_id + " cannot rejoin while battery safety is active";
        return false;
      }
      if ((*found)->operator_enabled == enabled && !(enabled && (*found)->safety_excluded)) {
        message = drone_id + (enabled ? " is already in the swarm" : " is already OFF");
        return false;
      }
      (*found)->operator_enabled = enabled;
      invalidatePlannedRoutesLocked();
      if (enabled) {
        (*found)->safety_excluded = false;
        (*found)->return_home_active = false;
      } else {
        const auto auxiliary = auxiliary_routes_.find(drone_id);
        if (auxiliary != auxiliary_routes_.end()) {
          mission_id = auxiliary->second.mission_id;
          revision = auxiliary->second.revision;
          auxiliary_routes_.erase(auxiliary);
          canceled_route = true;
        } else {
          mission_id = active_mission_id_;
          revision = active_revision_;
        }
        canceled_route = detachDroneRouteLocked(drone_id, true) || canceled_route;
        (*found)->return_home_active = false;
      }
      status_message_ = enabled ? drone_id + " rejoined the swarm" :
        drone_id + " marked OFF and excluded from routing";
      message = status_message_;
    }

    if (!enabled && canceled_route) {
      publishCancelCommand(mission_id, revision, {drone_id});
      message += "; its unfinished targets were returned to pending";
    }
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return true;
  }

  bool setDroneLidarRange(
    const std::string & drone_id,
    double range_m,
    std::string & message)
  {
    if (drone_id.empty()) {
      message = "drone_id is required for a LiDAR range command";
      return false;
    }
    if (!std::isfinite(range_m) || range_m < min_lidar_range_m_ ||
      range_m > max_lidar_range_m_)
    {
      std::ostringstream reason;
      reason << "LiDAR range must be between " << min_lidar_range_m_ << " and "
             << max_lidar_range_m_ << " m";
      message = reason.str();
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = std::find_if(
        drones_.begin(), drones_.end(),
        [&drone_id](const auto & drone) {return drone->id == drone_id;});
      if (found == drones_.end()) {
        message = "unknown drone: " + drone_id;
        return false;
      }
      if (!(*found)->has_lidar) {
        message = drone_id + " does not report a LiDAR";
        return false;
      }
      (*found)->lidar_range_m = range_m;
      std::ostringstream confirmation;
      confirmation << "set " << drone_id << " active LiDAR/APF range to " << range_m << " m";
      message = confirmation.str();
      status_message_ = message;
    }

    publishLidarRangeCommand(drone_id, range_m);
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  bool landDrones(const std::string & selected_drone_id, std::string & message)
  {
    std::vector<std::string> selected_drones;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (dispatch_phase_ != DispatchPhase::IDLE) {
        message = "cannot land while route feedback is pending";
        return false;
      }
      const auto current_time = now();
      for (const auto & drone : drones_) {
        if (!selected_drone_id.empty() && drone->id != selected_drone_id) continue;
        const bool connected = drone->have_state && drone->state.position_valid &&
          (current_time - drone->last_update).seconds() <= drone_state_timeout_s_;
        if (connected && drone->operator_enabled) selected_drones.push_back(drone->id);
      }
      if (selected_drones.empty()) {
        message = selected_drone_id.empty() ?
          "no enabled and connected drones are ready to land" :
          "selected drone is unknown, OFF, disconnected, or not localized";
        return false;
      }
      for (const auto & drone_id : selected_drones) {
        detachDroneRouteLocked(drone_id, true);
        auxiliary_routes_.erase(drone_id);
        findDrone(drone_id)->return_home_active = false;
      }
      invalidatePlannedRoutesLocked();
    }
    publishFlightCommand(SwarmMissionCommand::LAND, selected_drones);
    message = "landing command sent to " + std::to_string(selected_drones.size()) + " drone(s)";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_message_ = message;
    }
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  bool sendFlightControlCommand(
    uint8_t command_type,
    const std::string & selected_drone_id,
    double takeoff_altitude_m,
    double takeoff_climb_speed_m_s,
    std::string & message)
  {
    if (command_type == SwarmMissionCommand::TAKEOFF &&
      (!std::isfinite(takeoff_altitude_m) || takeoff_altitude_m <= min_altitude_m_ ||
      !std::isfinite(takeoff_climb_speed_m_s) || takeoff_climb_speed_m_s <= 0.0))
    {
      message = "takeoff altitude and climb speed must be valid positive values";
      return false;
    }

    std::vector<std::string> selected_drones;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto current_time = now();
      for (const auto & drone : drones_) {
        if (!selected_drone_id.empty() && drone->id != selected_drone_id) continue;
        if (droneHealthy(*drone, current_time)) selected_drones.push_back(drone->id);
      }
    }
    if (selected_drones.empty()) {
      message = selected_drone_id.empty() ?
        "no enabled and connected drones are ready" :
        "selected drone is unknown, OFF, disconnected, or not localized";
      return false;
    }

    publishFlightCommand(
      command_type, selected_drones, takeoff_altitude_m, takeoff_climb_speed_m_s);
    const std::string command_name = command_type == SwarmMissionCommand::ARM ?
      "arm" : "takeoff";
    message = command_name + " command sent to " +
      std::to_string(selected_drones.size()) + " drone(s)";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_message_ = message;
    }
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  void removeTarget(
    const RemoveSwarmTarget::Request::SharedPtr request,
    RemoveSwarmTarget::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dispatch_phase_ != DispatchPhase::IDLE) {
      response->message = "cannot remove a target while broadcast feedback is pending";
      response->pending_target_count = pending_targets_.size();
      return;
    }
    const auto target = std::find_if(
      pending_targets_.begin(), pending_targets_.end(),
      [&request](const TargetRecord & candidate) {return candidate.id == request->target_id;});
    if (target == pending_targets_.end()) {
      response->message = "pending target " + std::to_string(request->target_id) + " was not found";
      response->pending_target_count = pending_targets_.size();
      return;
    }
    pending_targets_.erase(target);
    invalidatePlannedRoutesLocked();
    response->removed = true;
    response->pending_target_count = pending_targets_.size();
    response->message = "removed pending target " + std::to_string(request->target_id);
    status_message_ = response->message;
  }

  bool droneHealthy(const DroneRecord & drone, const rclcpp::Time & current_time) const
  {
    if (!drone.operator_enabled) return false;
    if (drone.safety_excluded || !drone.battery_available_for_tasks) return false;
    const bool state_fresh = drone.have_state && drone.state.position_valid &&
      (current_time - drone.last_update).seconds() <= drone_state_timeout_s_;
    if (!state_fresh) return false;
    if (!require_registration_) return true;
    return heartbeatFreshLocked(drone, std::chrono::steady_clock::now()) &&
           drone.px4_ready && drone.localized && drone.navigation_ready;
  }

  double targetSpeed(const DroneRecord & drone, const TargetRecord & target) const
  {
    if (drone.has_speed_override) return std::max(0.2, drone.speed_override_m_s);
    if (target.cruise_speed_m_s > 0.0) return target.cruise_speed_m_s;
    return home_cruise_speed_m_s_;
  }

  static FlightPoint flightPoint(const geometry_msgs::msg::Point & point)
  {
    return {point.x, point.y, point.z};
  }

  FlightState flightState(const DroneRecord & drone) const
  {
    return {
      flightPoint(drone.state.position_enu),
      drone.state.heading_ned_rad,
      drone.state.vehicle_mode == VehicleState::MODE_FIXED_WING};
  }

  FlightTarget flightTarget(const DroneRecord & drone, const TargetRecord & target) const
  {
    return {
      flightPoint(target.target.pose.position),
      targetSpeed(drone, target),
      target.use_fixed_wing};
  }

  FlightTarget homeTarget(const DroneRecord & drone) const
  {
    geometry_msgs::msg::Point home = drone.geofence_origin;
    home.z += home_altitude_above_origin_m_;
    return {flightPoint(home), home_cruise_speed_m_s_, home_use_fixed_wing_};
  }

  double remainingMissionTime(
    const DroneRecord & drone,
    const std::map<std::string, RouteRecord> & routes) const
  {
    const auto found = routes.find(drone.id);
    if (found == routes.end()) return 0.0;
    const auto & route = found->second;
    const std::size_t active = std::min<std::size_t>(
      route.active_target_index, route.targets.size());
    if (active >= route.targets.size()) return 0.0;

    double result = flight_cost_model_->estimateFromState(
      flightState(drone), flightTarget(drone, route.targets[active])).time_s;
    FlightPoint incoming_reference = flightPoint(drone.state.position_enu);
    for (std::size_t index = active + 1; index < route.targets.size(); ++index) {
      const auto from = flightTarget(drone, route.targets[index - 1]);
      result += flight_cost_model_->estimateBetweenTargets(
        incoming_reference, from, flightTarget(drone, route.targets[index])).time_s;
      incoming_reference = from.position;
    }
    return result;
  }

  double batteryPenalty(const DroneRecord & drone) const
  {
    if (!drone.battery_valid || drone.battery_remaining_pct >= battery_low_pct_) return 0.0;
    const double span = std::max(1.0, battery_low_pct_ - battery_critical_pct_);
    const double normalized = std::clamp(
      (battery_low_pct_ - drone.battery_remaining_pct) / span, 0.0, 1.0);
    return battery_penalty_max_s_ * normalized * normalized;
  }

  bool calculateRoutes(bool recalculate, std::string & message)
  {
    std::vector<TargetRecord> targets;
    std::vector<std::shared_ptr<DroneRecord>> eligible_drones;
    std::map<std::string, RouteRecord> previous_routes;
    const auto current_time = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (dispatch_phase_ != DispatchPhase::IDLE) {
        message = "a mission broadcast is already waiting for drone feedback";
        return false;
      }
      if (!recalculate && !current_routes_.empty()) {
        message = "an active mission exists; use RECALCULATE";
        return false;
      }
      if (recalculate && current_routes_.empty()) {
        message = "there is no active mission to recalculate";
        return false;
      }
      if (recalculate) {
        for (const auto & active : active_targets_) targets.push_back(active.second);
      }
      previous_routes = current_routes_;
      targets.insert(targets.end(), pending_targets_.begin(), pending_targets_.end());
      if (targets.empty()) {
        message = "there are no targets to route";
        return false;
      }

      for (const auto & drone : drones_) {
        const bool owns_active_route = current_routes_.count(drone->id) != 0;
        if (droneHealthy(*drone, current_time) && (!drone->busy || owns_active_route)) {
          eligible_drones.push_back(drone);
        }
      }
    }

    if (eligible_drones.empty()) {
      message = "no connected, localized, and available drones were found";
      return false;
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> start_costs(
      eligible_drones.size(), std::vector<double>(targets.size(), infinity));
    std::vector<std::vector<std::vector<double>>> target_costs(
      eligible_drones.size(),
      std::vector<std::vector<double>>(
        targets.size(), std::vector<double>(targets.size(), 0.0)));
    std::vector<std::vector<double>> return_costs(
      eligible_drones.size(), std::vector<double>(targets.size(), infinity));
    std::vector<double> route_base_costs(eligible_drones.size(), 0.0);
    std::vector<double> maximum_route_travel_costs(
      eligible_drones.size(), infinity);
    RouteResourceCosts energy_costs;
    energy_costs.starts = std::vector<std::vector<double>>(
      eligible_drones.size(), std::vector<double>(targets.size(), infinity));
    energy_costs.legs = std::vector<std::vector<std::vector<double>>>(
      eligible_drones.size(),
      std::vector<std::vector<double>>(
        targets.size(), std::vector<double>(targets.size(), 0.0)));
    energy_costs.returns = std::vector<std::vector<double>>(
      eligible_drones.size(), std::vector<double>(targets.size(), infinity));
    energy_costs.maximum = std::vector<double>(eligible_drones.size(), infinity);
    for (std::size_t drone = 0; drone < eligible_drones.size(); ++drone) {
      const auto & drone_record = *eligible_drones[drone];
      route_base_costs[drone] = remainingMissionTime(drone_record, previous_routes) +
        batteryPenalty(drone_record) +
        (previous_routes.count(drone_record.id) != 0 ? active_route_penalty_s_ : 0.0);
      if (drone_record.battery_valid &&
        std::isfinite(drone_record.battery_time_remaining_s) &&
        drone_record.battery_time_remaining_s >= 0.0)
      {
        maximum_route_travel_costs[drone] = std::max(
          0.0, drone_record.battery_time_remaining_s - battery_reserve_s_);
      }
      if (drone_record.battery_valid &&
        std::isfinite(drone_record.battery_capacity_wh) &&
        std::isfinite(drone_record.battery_remaining_energy_wh) &&
        drone_record.battery_capacity_wh > 0.0 &&
        drone_record.battery_remaining_energy_wh >= 0.0)
      {
        const double reserve_wh = drone_record.battery_capacity_wh *
          battery_energy_reserve_pct_ / 100.0;
        energy_costs.maximum[drone] = std::max(
          0.0, drone_record.battery_remaining_energy_wh - reserve_wh);
      }
      for (std::size_t target = 0; target < targets.size(); ++target) {
        if (targetValidForDrone(targets[target], drone_record)) {
          const auto target_flight = flightTarget(drone_record, targets[target]);
          const auto start_estimate = flight_cost_model_->estimateFromState(
            flightState(drone_record), target_flight);
          const auto return_estimate = flight_cost_model_->estimateBetweenTargets(
            flightPoint(drone_record.state.position_enu), target_flight,
            homeTarget(drone_record));
          start_costs[drone][target] = start_estimate.time_s;
          return_costs[drone][target] = return_estimate.time_s;
          energy_costs.starts[drone][target] = start_estimate.energy_wh;
          energy_costs.returns[drone][target] = return_estimate.energy_wh;
        }
      }
      for (std::size_t from = 0; from < targets.size(); ++from) {
        for (std::size_t to = 0; to < targets.size(); ++to) {
          if (from == to) continue;
          const auto estimate = flight_cost_model_->estimateBetweenTargets(
            flightPoint(drone_record.state.position_enu),
            flightTarget(drone_record, targets[from]),
            flightTarget(drone_record, targets[to]));
          target_costs[drone][from][to] = estimate.time_s;
          energy_costs.legs[drone][from][to] = estimate.energy_wh;
        }
      }
    }

    std::vector<RoutePlan> solved;
    try {
      solved = RouteSolver::solve(
        start_costs, target_costs, route_base_costs,
        maximum_route_travel_costs, return_costs, true, route_improvement_passes_,
        energy_costs);
    } catch (const std::exception & exception) {
      message = std::string("route optimization failed: ") + exception.what();
      return false;
    }

    std::map<std::string, RouteRecord> plan;
    for (const auto & route_plan : solved) {
      const auto & drone = eligible_drones[route_plan.drone_index];
      RouteRecord route;
      route.token = next_route_token_++;
      route.route_id = route.token;
      route.drone_id = drone->id;
      route.total_cost = route_plan.total_cost;
      FlightPoint incoming_reference = flightPoint(drone->state.position_enu);
      FlightTarget previous_target;
      bool have_previous_target = false;
      for (const auto target_index : route_plan.target_indices) {
        route.targets.push_back(targets[target_index]);
        if (drone->has_speed_override) {
          route.targets.back().cruise_speed_m_s = drone->speed_override_m_s;
        }
        const auto current_target = flightTarget(*drone, targets[target_index]);
        const auto estimate = have_previous_target ?
          flight_cost_model_->estimateBetweenTargets(
          incoming_reference, previous_target, current_target) :
          flight_cost_model_->estimateFromState(flightState(*drone), current_target);
        route.leg_costs.push_back(estimate.time_s);
        if (have_previous_target) incoming_reference = previous_target.position;
        previous_target = current_target;
        have_previous_target = true;
      }

      const auto previous = previous_routes.find(route.drone_id);
      if (previous != previous_routes.end() && sameRemainingRoute(previous->second, route)) {
        route = previous->second;
        route.message = "unchanged route waiting for revision acknowledgment";
      }
      route.accepted = false;
      plan[route.drone_id] = std::move(route);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      planned_routes_ = plan;
      planned_recalculation_ = recalculate;
      status_message_ = recalculate ?
        "recalculated route preview ready; press START MISSION to dispatch" :
        "route preview ready; press START MISSION to dispatch";
      message = status_message_;
    }

    logRoutes(eligible_drones, plan, recalculate);
    return true;
  }

  bool dispatchPlannedMission(std::string & message)
  {
    std::map<std::string, RouteRecord> plan;
    uint64_t command_id = 0;
    uint64_t mission_id = 0;
    uint32_t revision = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (dispatch_phase_ != DispatchPhase::IDLE) {
        message = "a mission broadcast is already waiting for drone feedback";
        return false;
      }
      if (planned_routes_.empty()) {
        message = "there is no calculated route plan; press CALCULATE first";
        return false;
      }
      if (planned_recalculation_ != !current_routes_.empty()) {
        message = "the calculated plan no longer matches the mission state; calculate again";
        return false;
      }

      const auto current_time = now();
      for (const auto & entry : planned_routes_) {
        const auto drone = std::find_if(
          drones_.begin(), drones_.end(),
          [&entry](const auto & candidate) {return candidate->id == entry.first;});
        const bool owns_active_route = current_routes_.count(entry.first) != 0;
        if (drone == drones_.end() || !droneHealthy(**drone, current_time) ||
          ((*drone)->busy && !owns_active_route))
        {
          message = "planned drone " + entry.first +
            " is no longer available; calculate again";
          return false;
        }
      }

      std::set<uint64_t> available_target_ids;
      for (const auto & target : pending_targets_) available_target_ids.insert(target.id);
      if (planned_recalculation_) {
        for (const auto & target : active_targets_) available_target_ids.insert(target.first);
      }
      std::set<uint64_t> planned_target_ids;
      for (const auto & entry : planned_routes_) {
        for (const auto & target : entry.second.targets) planned_target_ids.insert(target.id);
      }
      if (planned_target_ids != available_target_ids) {
        message = "the target buffer changed after calculation; calculate again";
        return false;
      }

      if (!planned_recalculation_) {
        active_mission_id_ = next_mission_id_++;
        active_revision_ = 1;
      } else {
        ++active_revision_;
      }
      active_command_id_ = next_command_id_++;
      command_id = active_command_id_;
      mission_id = active_mission_id_;
      revision = active_revision_;
      plan = planned_routes_;
      provisional_routes_ = planned_routes_;
      planned_routes_.clear();
      recalculation_in_progress_ = planned_recalculation_;
      planned_recalculation_ = false;
      dispatch_phase_ = DispatchPhase::WAITING_FOR_FEEDBACK;
      dispatch_deadline_ = now() + rclcpp::Duration::from_seconds(feedback_timeout_s_);
      for (const auto & route : provisional_routes_) findDrone(route.first)->busy = true;
      status_message_ = recalculation_in_progress_ ?
        "recalculated mission broadcast; waiting for drone feedback" :
        "mission broadcast; waiting for drone feedback";
      message = status_message_;
    }

    publishExecuteCommand(command_id, mission_id, revision, plan);
    return true;
  }

  bool sameRemainingRoute(const RouteRecord & current, const RouteRecord & replacement) const
  {
    const auto current_begin = std::min<std::size_t>(
      current.completed_target_count, current.targets.size());
    if (current.targets.size() - current_begin != replacement.targets.size()) return false;
    for (std::size_t index = 0; index < replacement.targets.size(); ++index) {
      if (current.targets[current_begin + index].id != replacement.targets[index].id) {
        return false;
      }
    }
    return true;
  }

  bool startHomeMission(
    const std::string & selected_drone_id,
    std::string & message,
    uint8_t route_purpose = SwarmRoute::PURPOSE_HOME)
  {
    std::map<std::string, RouteRecord> plan;
    uint64_t command_id = 0;
    uint64_t mission_id = 0;
    uint32_t revision = 1;
    const auto current_time = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (dispatch_phase_ != DispatchPhase::IDLE) {
        message = "cannot return home while route acceptance feedback is pending";
        return false;
      }

      for (const auto & drone : drones_) {
        if (!selected_drone_id.empty() && drone->id != selected_drone_id) continue;
        const bool connected = drone->have_state && drone->state.position_valid &&
          (current_time - drone->last_update).seconds() <= drone_state_timeout_s_;
        if (!connected || !drone->operator_enabled || !drone->have_geofence_origin) continue;

        TargetRecord approach;
        approach.id = next_target_id_++;
        approach.target.header.stamp = current_time;
        approach.target.header.frame_id = map_frame_;
        approach.target.pose.position.x = drone->geofence_origin.x;
        approach.target.pose.position.y = drone->geofence_origin.y;
        approach.target.pose.position.z =
          drone->geofence_origin.z + home_altitude_above_origin_m_;
        approach.target.pose.orientation.w = 1.0;
        approach.cruise_speed_m_s = home_cruise_speed_m_s_;
        if (drone->has_speed_override) {
          approach.cruise_speed_m_s = drone->speed_override_m_s;
        }
        approach.use_fixed_wing = home_use_fixed_wing_;
        approach.preserve_on_cancel = false;

        TargetRecord precision = approach;
        precision.id = next_target_id_++;
        precision.cruise_speed_m_s = home_precision_speed_m_s_;
        precision.use_fixed_wing = false;

        RouteRecord route;
        route.token = next_route_token_++;
        route.route_id = route.token;
        route.drone_id = drone->id;
        route.purpose = route_purpose;
        route.targets = {approach, precision};
        const auto approach_cost = flight_cost_model_->estimateFromState(
          flightState(*drone), flightTarget(*drone, approach)).time_s;
        const auto precision_cost = flight_cost_model_->estimateBetweenTargets(
          flightPoint(drone->state.position_enu), flightTarget(*drone, approach),
          flightTarget(*drone, precision)).time_s;
        route.total_cost = approach_cost + precision_cost;
        route.leg_costs = {approach_cost, precision_cost};
        plan[drone->id] = std::move(route);
      }
      if (plan.empty()) {
        message = selected_drone_id.empty() ?
          "no connected and localized drones with a recorded home position were found" :
          "selected drone is not connected, localized, or missing its home position";
        return false;
      }

      command_id = next_command_id_++;
      mission_id = next_mission_id_++;
      invalidatePlannedRoutesLocked();
      for (const auto & route : plan) {
        detachDroneRouteLocked(route.first, true);
        auxiliary_routes_[route.first] = {command_id, mission_id, revision, route.second};
        auto drone = findDrone(route.first);
        drone->busy = true;
        drone->return_home_active = true;
      }
      status_message_ = "targeted return-home routes broadcast";
      message = status_message_;
    }

    RCLCPP_INFO(get_logger(), "Returning %zu drone(s) to their recorded home positions", plan.size());
    publishExecuteCommand(command_id, mission_id, revision, plan);
    return true;
  }

  void logRoutes(
    const std::vector<std::shared_ptr<DroneRecord>> & drones,
    const std::map<std::string, RouteRecord> & routes,
    bool recalculate)
  {
    RCLCPP_INFO(
      get_logger(), "%s with %zu route(s) across %zu healthy drone(s)",
      recalculate ? "Recalculating" : "Calculating", routes.size(), drones.size());
    for (const auto & entry : routes) {
      std::ostringstream description;
      description << "Route " << entry.first << " cost=" << entry.second.total_cost << ':';
      for (const auto & target : entry.second.targets) description << ' ' << target.id;
      RCLCPP_INFO(get_logger(), "%s", description.str().c_str());
    }
  }

  void publishExecuteCommand(
    uint64_t command_id,
    uint64_t mission_id,
    uint32_t revision,
    const std::map<std::string, RouteRecord> & routes)
  {
    SwarmMissionCommand command;
    command.header.stamp = now();
    command.header.frame_id = map_frame_;
    command.command_id = command_id;
    command.mission_id = mission_id;
    command.revision = revision;
    command.command = SwarmMissionCommand::EXECUTE;
    for (const auto & entry : routes) {
      SwarmRoute route;
      route.drone_id = entry.first;
      route.route_id = entry.second.route_id;
      route.purpose = entry.second.purpose;
      for (const auto & target : entry.second.targets) {
        RouteTarget route_target;
        route_target.target_id = target.id;
        route_target.target = target.target;
        route_target.cruise_speed_m_s = target.cruise_speed_m_s;
        route_target.use_fixed_wing = target.use_fixed_wing;
        route.targets.push_back(std::move(route_target));
      }
      command.routes.push_back(std::move(route));
    }
    mission_command_pub_->publish(command);
    RCLCPP_INFO(
      get_logger(), "Broadcast EXECUTE command %lu mission %lu revision %u to %zu route(s)",
      command_id, mission_id, revision, routes.size());
  }

  void publishCancelCommand(
    uint64_t mission_id,
    uint32_t revision,
    const std::vector<std::string> & target_drone_ids = {})
  {
    if (mission_id == 0) return;
    SwarmMissionCommand command;
    command.header.stamp = now();
    command.header.frame_id = map_frame_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command.command_id = next_command_id_++;
    }
    command.mission_id = mission_id;
    command.revision = revision;
    command.command = SwarmMissionCommand::CANCEL;
    command.target_drone_ids = target_drone_ids;
    mission_command_pub_->publish(command);
    RCLCPP_WARN(
      get_logger(), "Broadcast CANCEL command %lu for mission %lu revision %u",
      command.command_id, mission_id, revision);
  }

  void publishSpeedCommand(
    const std::string & drone_id,
    double speed_m_s,
    bool clear_override)
  {
    SwarmMissionCommand command;
    command.header.stamp = now();
    command.header.frame_id = map_frame_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command.command_id = next_command_id_++;
      command.mission_id = active_mission_id_;
      command.revision = active_revision_;
    }
    command.command = SwarmMissionCommand::SET_SPEED;
    command.target_drone_ids.push_back(drone_id);
    command.cruise_speed_m_s = clear_override ? 0.0 : speed_m_s;
    mission_command_pub_->publish(command);
  }

  void publishLidarRangeCommand(const std::string & drone_id, double range_m)
  {
    SwarmMissionCommand command;
    command.header.stamp = now();
    command.header.frame_id = map_frame_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command.command_id = next_command_id_++;
      command.mission_id = active_mission_id_;
      command.revision = active_revision_;
    }
    command.command = SwarmMissionCommand::SET_LIDAR_RANGE;
    command.target_drone_ids.push_back(drone_id);
    command.lidar_range_m = range_m;
    mission_command_pub_->publish(command);
  }

  void publishFlightCommand(
    uint8_t command_type,
    const std::vector<std::string> & target_drone_ids,
    double takeoff_altitude_m = 0.0,
    double takeoff_climb_speed_m_s = 0.0)
  {
    SwarmMissionCommand command;
    command.header.stamp = now();
    command.header.frame_id = map_frame_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command.command_id = next_command_id_++;
      command.mission_id = active_mission_id_;
      command.revision = active_revision_;
    }
    command.command = command_type;
    command.target_drone_ids = target_drone_ids;
    command.takeoff_altitude_m = takeoff_altitude_m;
    command.takeoff_climb_speed_m_s = takeoff_climb_speed_m_s;
    mission_command_pub_->publish(command);
  }

  std::shared_ptr<DroneRecord> findDrone(const std::string & drone_id) const
  {
    const auto found = std::find_if(
      drones_.begin(), drones_.end(),
      [&drone_id](const auto & drone) {return drone->id == drone_id;});
    if (found == drones_.end()) throw std::runtime_error("unknown drone: " + drone_id);
    return *found;
  }

  RouteRecord * findRouteLocked(
    std::map<std::string, RouteRecord> & routes,
    const SwarmMissionFeedback & feedback)
  {
    const auto found = routes.find(feedback.drone_id);
    if (found == routes.end() || found->second.route_id != feedback.route_id) return nullptr;
    return &found->second;
  }

  void updateRouteFromFeedbackLocked(
    RouteRecord & route,
    const SwarmMissionFeedback & feedback)
  {
    route.active_target_id = feedback.active_target_id;
    route.active_target_index = feedback.active_target_index;
    route.completed_target_count = feedback.completed_target_count;
    route.distance_remaining_m = feedback.remaining_distance_m;
    route.message = feedback.message;
    switch (feedback.state) {
      case SwarmMissionFeedback::ACCEPTED:
        route.accepted = true;
        route.state = SwarmAssignment::ACCEPTED;
        break;
      case SwarmMissionFeedback::EXECUTING:
        route.accepted = true;
        route.state = SwarmAssignment::EXECUTING;
        break;
      case SwarmMissionFeedback::SUCCEEDED:
        route.accepted = true;
        route.finished = true;
        route.state = SwarmAssignment::SUCCEEDED;
        route.completed_target_count = route.targets.size();
        break;
      case SwarmMissionFeedback::CANCELED:
        route.state = SwarmAssignment::CANCELED;
        break;
      case SwarmMissionFeedback::REJECTED:
        route.state = SwarmAssignment::REJECTED;
        break;
      case SwarmMissionFeedback::FAILED:
        route.state = SwarmAssignment::FAILED;
        break;
      default:
        break;
    }
    removeCompletedTargetsLocked(route);
  }

  void onMissionFeedback(const SwarmMissionFeedback::SharedPtr feedback)
  {
    bool commit = false;
    bool provisional_failure = false;
    bool mission_failure = false;
    bool mission_complete = false;
    std::string failure_reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto auxiliary = auxiliary_routes_.find(feedback->drone_id);
      if (auxiliary != auxiliary_routes_.end() &&
        feedback->command_id == auxiliary->second.command_id &&
        feedback->mission_id == auxiliary->second.mission_id &&
        feedback->revision == auxiliary->second.revision &&
        feedback->route_id == auxiliary->second.route.route_id)
      {
        updateRouteFromFeedbackLocked(auxiliary->second.route, *feedback);
        const bool terminal = feedback->state == SwarmMissionFeedback::SUCCEEDED ||
          feedback->state == SwarmMissionFeedback::FAILED ||
          feedback->state == SwarmMissionFeedback::REJECTED ||
          feedback->state == SwarmMissionFeedback::CANCELED;
        if (terminal) {
          auto drone = findDrone(feedback->drone_id);
          drone->busy = false;
          drone->return_home_active = false;
          status_message_ = feedback->drone_id +
            (feedback->state == SwarmMissionFeedback::SUCCEEDED ?
            " reached home" : " home route ended: " + feedback->message);
          auxiliary_routes_.erase(auxiliary);
        }
        return;
      }
      const bool current_broadcast = feedback->command_id == active_command_id_ &&
        feedback->mission_id == active_mission_id_ && feedback->revision == active_revision_;

      if (dispatch_phase_ == DispatchPhase::WAITING_FOR_FEEDBACK && current_broadcast) {
        auto * route = findRouteLocked(provisional_routes_, *feedback);
        if (!route) return;
        updateRouteFromFeedbackLocked(*route, *feedback);
        provisional_failure = feedback->state == SwarmMissionFeedback::REJECTED ||
          feedback->state == SwarmMissionFeedback::FAILED ||
          feedback->state == SwarmMissionFeedback::CANCELED;
        if (provisional_failure) {
          failure_reason = "drone " + feedback->drone_id +
            " rejected broadcast route: " + feedback->message;
        } else {
          commit = std::all_of(
            provisional_routes_.begin(), provisional_routes_.end(),
            [](const auto & entry) {return entry.second.accepted;});
        }
      } else if (feedback->mission_id == active_mission_id_ &&
        feedback->revision == active_revision_)
      {
        auto * route = findRouteLocked(current_routes_, *feedback);
        if (!route) return;
        updateRouteFromFeedbackLocked(*route, *feedback);
        mission_failure = feedback->state == SwarmMissionFeedback::FAILED ||
          feedback->state == SwarmMissionFeedback::REJECTED ||
          feedback->state == SwarmMissionFeedback::CANCELED;
        if (mission_failure) {
          failure_reason = "drone " + feedback->drone_id +
            " route failed: " + feedback->message;
        } else {
          mission_complete = !current_routes_.empty() && std::all_of(
            current_routes_.begin(), current_routes_.end(),
            [](const auto & entry) {return entry.second.finished;});
        }
      }
    }

    if (provisional_failure) {
      failProvisional(failure_reason);
    } else if (commit) {
      commitProvisional();
    } else if (mission_failure) {
      abortActiveMission(failure_reason);
    } else if (mission_complete) {
      finishMission();
    }
  }

  void removeCompletedTargetsLocked(const RouteRecord & route)
  {
    const auto completed = std::min<std::size_t>(
      route.completed_target_count, route.targets.size());
    for (std::size_t index = 0; index < completed; ++index) {
      active_targets_.erase(route.targets[index].id);
    }
  }

  void commitProvisional()
  {
    bool all_finished = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (provisional_routes_.empty() ||
        dispatch_phase_ != DispatchPhase::WAITING_FOR_FEEDBACK) return;
      for (const auto & route : provisional_routes_) {
        for (std::size_t index = route.second.completed_target_count;
          index < route.second.targets.size(); ++index)
        {
          const auto & target = route.second.targets[index];
          active_targets_[target.id] = target;
          pending_targets_.erase(
            std::remove_if(
              pending_targets_.begin(), pending_targets_.end(),
              [&target](const TargetRecord & pending) {return pending.id == target.id;}),
            pending_targets_.end());
        }
      }
      current_routes_ = std::move(provisional_routes_);
      provisional_routes_.clear();
      dispatch_phase_ = DispatchPhase::IDLE;
      recalculation_in_progress_ = false;
      status_message_ = "broadcast routes accepted by all assigned drones";
      all_finished = std::all_of(
        current_routes_.begin(), current_routes_.end(),
        [](const auto & route) {return route.second.finished;});
    }
    RCLCPP_INFO(get_logger(), "%s", status_message_.c_str());
    if (all_finished) finishMission();
  }

  void failProvisional(const std::string & reason)
  {
    bool was_recalculation = false;
    uint64_t mission_id = 0;
    uint32_t revision = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (provisional_routes_.empty()) return;
      was_recalculation = recalculation_in_progress_;
      mission_id = active_mission_id_;
      revision = active_revision_;
      provisional_routes_.clear();
      dispatch_phase_ = DispatchPhase::IDLE;
      recalculation_in_progress_ = false;
      status_message_ = reason + "; pending targets preserved";
    }
    publishCancelCommand(mission_id, revision);
    if (was_recalculation) {
      abortActiveMission(reason + "; recalculation could not be committed", false);
    } else {
      releaseUnassignedDrones();
      RCLCPP_ERROR(get_logger(), "Broadcast dispatch failed: %s", status_message_.c_str());
    }
  }

  bool clearPendingTargets(std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dispatch_phase_ != DispatchPhase::IDLE) {
      message = "cannot clear targets while broadcast feedback is pending";
      return false;
    }
    const auto count = pending_targets_.size();
    pending_targets_.clear();
    invalidatePlannedRoutesLocked();
    message = "cleared " + std::to_string(count) + " pending target(s)";
    status_message_ = message;
    return true;
  }

  void invalidatePlannedRoutesLocked()
  {
    planned_routes_.clear();
    planned_recalculation_ = false;
  }

  bool cancelActiveMission(std::string & message)
  {
    uint64_t mission_id = 0;
    uint32_t revision = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (current_routes_.empty() && provisional_routes_.empty()) {
        message = "there is no active mission to cancel";
        return false;
      }
      mission_id = active_mission_id_;
      revision = active_revision_;
      restoreActiveTargetsLocked();
      current_routes_.clear();
      provisional_routes_.clear();
      invalidatePlannedRoutesLocked();
      dispatch_phase_ = DispatchPhase::IDLE;
      recalculation_in_progress_ = false;
      message = "active mission canceled; unfinished targets returned to pending buffer";
      status_message_ = message;
    }
    publishCancelCommand(mission_id, revision);
    releaseUnassignedDrones();
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return true;
  }

  void finishMission()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & drone : drones_) {
      drone->busy = auxiliary_routes_.count(drone->id) != 0;
    }
    current_routes_.clear();
    active_targets_.clear();
    invalidatePlannedRoutesLocked();
    status_message_ = "swarm mission succeeded";
    RCLCPP_INFO(get_logger(), "%s", status_message_.c_str());
  }

  void abortActiveMission(const std::string & reason, bool publish_cancel = true)
  {
    uint64_t mission_id = 0;
    uint32_t revision = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mission_id = active_mission_id_;
      revision = active_revision_;
      restoreActiveTargetsLocked();
      current_routes_.clear();
      provisional_routes_.clear();
      invalidatePlannedRoutesLocked();
      dispatch_phase_ = DispatchPhase::IDLE;
      recalculation_in_progress_ = false;
      status_message_ = reason + "; unfinished targets returned to pending";
    }
    if (publish_cancel) publishCancelCommand(mission_id, revision);
    releaseUnassignedDrones();
    RCLCPP_ERROR(get_logger(), "%s", status_message_.c_str());
  }

  void restoreActiveTargetsLocked()
  {
    for (const auto & active : active_targets_) {
      if (!active.second.preserve_on_cancel) continue;
      const auto pending = std::find_if(
        pending_targets_.begin(), pending_targets_.end(),
        [&active](const TargetRecord & target) {return target.id == active.first;});
      if (pending == pending_targets_.end()) pending_targets_.push_back(active.second);
    }
    active_targets_.clear();
  }

  bool detachDroneRouteLocked(const std::string & drone_id, bool preserve_unfinished)
  {
    bool detached = false;
    const auto current = current_routes_.find(drone_id);
    if (current != current_routes_.end()) {
      const auto & route = current->second;
      const std::size_t completed = std::min<std::size_t>(
        route.completed_target_count, route.targets.size());
      for (std::size_t index = 0; index < route.targets.size(); ++index) {
        const auto & target = route.targets[index];
        active_targets_.erase(target.id);
        if (!preserve_unfinished || index < completed || !target.preserve_on_cancel) continue;
        const auto pending = std::find_if(
          pending_targets_.begin(), pending_targets_.end(),
          [&target](const TargetRecord & candidate) {return candidate.id == target.id;});
        if (pending == pending_targets_.end()) pending_targets_.push_back(target);
      }
      current_routes_.erase(current);
      detached = true;
    }
    if (provisional_routes_.erase(drone_id) != 0) detached = true;
    const auto drone = std::find_if(
      drones_.begin(), drones_.end(),
      [&drone_id](const auto & candidate) {return candidate->id == drone_id;});
    if (drone != drones_.end()) (*drone)->busy = false;
    return detached;
  }

  void handleSafetyEvent(const SafetyEvent & event)
  {
    if (event.battery_state == SwarmDroneHeartbeat::BATTERY_STATE_EMERGENCY_LAND) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        detachDroneRouteLocked(event.drone_id, true);
        auxiliary_routes_.erase(event.drone_id);
        auto drone = findDrone(event.drone_id);
        drone->safety_excluded = true;
        drone->return_home_active = false;
        status_message_ = event.drone_id +
          " entered battery emergency; targeted LAND sent and tasks recovered";
      }
      publishFlightCommand(SwarmMissionCommand::LAND, {event.drone_id});
      RCLCPP_ERROR(get_logger(), "%s", status_message_.c_str());
    } else {
      std::string home_message;
      const bool home_dispatched = startHomeMission(
        event.drone_id, home_message, SwarmRoute::PURPOSE_LOW_BATTERY_RTH);
      if (!home_dispatched)
      {
        RCLCPP_ERROR(
          get_logger(), "Could not dispatch low-battery RTH for %s: %s",
          event.drone_id.c_str(), home_message.c_str());
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto drone = findDrone(event.drone_id);
        drone->safety_excluded = true;
        drone->return_home_active = home_dispatched;
      }
    }

    bool have_pending = false;
    bool have_active_routes = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_pending = !pending_targets_.empty();
      have_active_routes = !current_routes_.empty();
    }
    if (!have_pending) return;

    std::string routing_message;
    if (!calculateRoutes(have_active_routes, routing_message) ||
      !dispatchPlannedMission(routing_message))
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_message_ += "; recovered tasks remain pending: " + routing_message;
      RCLCPP_WARN(get_logger(), "%s", status_message_.c_str());
    }
  }

  void releaseUnassignedDrones()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & drone : drones_) {
      if (current_routes_.count(drone->id) == 0 &&
        provisional_routes_.count(drone->id) == 0 &&
        auxiliary_routes_.count(drone->id) == 0)
      {
        drone->busy = false;
      }
    }
  }

  void onTimer()
  {
    discoverDrones();
    bool dispatch_timed_out = false;
    bool have_safety_event = false;
    SafetyEvent safety_event;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dispatch_timed_out = dispatch_phase_ == DispatchPhase::WAITING_FOR_FEEDBACK &&
        now() >= dispatch_deadline_;
      if (dispatch_phase_ == DispatchPhase::IDLE && !pending_safety_events_.empty()) {
        safety_event = pending_safety_events_.front();
        pending_safety_events_.erase(pending_safety_events_.begin());
        have_safety_event = true;
      }
    }
    if (dispatch_timed_out) {
      failProvisional("timed out waiting for broadcast acceptance feedback");
    }
    if (have_safety_event) handleSafetyEvent(safety_event);
    publishState();
  }

  void publishState()
  {
    SwarmState state;
    state.header.stamp = now();
    state.header.frame_id = map_frame_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state.pending_target_count = pending_targets_.size();
      state.active_target_count = active_targets_.size();
      state.dispatch_in_progress = dispatch_phase_ != DispatchPhase::IDLE;
      state.mission_active = !current_routes_.empty();
      state.plan_ready = !planned_routes_.empty();
      state.plan_is_recalculation = planned_recalculation_;
      state.planned_drone_count = planned_routes_.size();
      state.status_message = status_message_;
      for (const auto & target : pending_targets_) {
        RouteTarget route_target;
        route_target.target_id = target.id;
        route_target.target = target.target;
        route_target.cruise_speed_m_s = target.cruise_speed_m_s;
        route_target.use_fixed_wing = target.use_fixed_wing;
        state.pending_targets.push_back(std::move(route_target));
      }
      for (const auto & active : active_targets_) {
        RouteTarget route_target;
        route_target.target_id = active.second.id;
        route_target.target = active.second.target;
        route_target.cruise_speed_m_s = active.second.cruise_speed_m_s;
        route_target.use_fixed_wing = active.second.use_fixed_wing;
        state.active_targets.push_back(std::move(route_target));
      }
      const auto current_time = now();
      state.available_drone_count = std::count_if(
        drones_.begin(), drones_.end(),
        [this, &current_time](const auto & drone) {
          return droneHealthy(*drone, current_time) && !drone->busy;
        });
      const auto steady_time = std::chrono::steady_clock::now();
      for (const auto & drone : drones_) {
        SwarmDroneState drone_state;
        drone_state.header = state.header;
        drone_state.drone_id = drone->id;
        drone_state.drone_namespace = drone->drone_namespace;
        drone_state.boot_id = drone->boot_id;
        drone_state.registered = drone->registered;
        drone_state.connected = require_registration_ ?
          heartbeatFreshLocked(*drone, steady_time) : drone->have_state;
        drone_state.localized = drone->state.position_valid &&
          (!require_registration_ || drone->localized);
        drone_state.navigation_ready = !require_registration_ || drone->navigation_ready;
        drone_state.operator_enabled = drone->operator_enabled;
        drone_state.busy = drone->busy;
        drone_state.available = droneHealthy(*drone, current_time) && !drone->busy;
        drone_state.armed = drone->state.armed;
        drone_state.offboard = drone->state.offboard;
        drone_state.has_lidar = drone->has_lidar;
        drone_state.supports_fixed_wing = drone->supports_fixed_wing;
        drone_state.supports_vtol = drone->supports_vtol;
        drone_state.has_speed_override = drone->has_speed_override;
        drone_state.speed_override_m_s = drone->speed_override_m_s;
        drone_state.lidar_range_m = drone->lidar_range_m;
        drone_state.battery_valid = drone->battery_valid;
        drone_state.battery_remaining_pct = drone->battery_remaining_pct;
        drone_state.battery_time_remaining_s = drone->battery_time_remaining_s;
        drone_state.battery_power_w = drone->battery_power_w;
        drone_state.battery_capacity_wh = drone->battery_capacity_wh;
        drone_state.battery_remaining_energy_wh = drone->battery_remaining_energy_wh;
        drone_state.battery_state = drone->battery_state;
        drone_state.safety_excluded = drone->safety_excluded;
        drone_state.return_home_active = drone->return_home_active;
        drone_state.last_update_age_sec = drone->have_state ?
          std::max(0.0, (current_time - drone->last_update).seconds()) :
          std::numeric_limits<double>::infinity();
        drone_state.position = drone->state.position_enu;
        state.drones.push_back(std::move(drone_state));
      }
      appendRouteState(current_routes_, state.assignments);
      appendRouteState(provisional_routes_, state.assignments);
      appendRouteState(planned_routes_, state.planned_assignments);
      for (const auto & auxiliary : auxiliary_routes_) {
        const std::map<std::string, RouteRecord> route{{auxiliary.first, auxiliary.second.route}};
        appendRouteState(route, state.assignments);
      }
    }
    state_pub_->publish(state);
  }

  void appendRouteState(
    const std::map<std::string, RouteRecord> & routes,
    std::vector<SwarmAssignment> & assignments)
  {
    for (const auto & entry : routes) {
      const auto & route = entry.second;
      for (std::size_t index = 0; index < route.targets.size(); ++index) {
        SwarmAssignment assignment;
        assignment.target_id = route.targets[index].id;
        assignment.drone_id = route.drone_id;
        assignment.target = route.targets[index].target;
        assignment.current_position = findDrone(entry.first)->state.position_enu;
        assignment.cost = index < route.leg_costs.size() ? route.leg_costs[index] : 0.0;
        assignment.route_total_cost = route.total_cost;
        assignment.distance_remaining_m = index == route.active_target_index ?
          route.distance_remaining_m : std::numeric_limits<double>::quiet_NaN();
        if (index < route.completed_target_count ||
          (route.finished && route.state == SwarmAssignment::SUCCEEDED))
        {
          assignment.state = SwarmAssignment::SUCCEEDED;
        } else if (index == route.active_target_index &&
          route.state == SwarmAssignment::EXECUTING)
        {
          assignment.state = SwarmAssignment::EXECUTING;
        } else if (route.state == SwarmAssignment::FAILED ||
          route.state == SwarmAssignment::CANCELED ||
          route.state == SwarmAssignment::REJECTED)
        {
          assignment.state = route.state;
        } else {
          assignment.state = route.state;
        }
        std::ostringstream message;
        message << "route step " << (index + 1) << '/' << route.targets.size()
                << " | route cost=" << route.total_cost << " | " << route.message;
        assignment.message = message.str();
        assignments.push_back(std::move(assignment));
      }
    }
  }

  std::mutex mutex_;
  std::string map_frame_ {"map"};
  double drone_state_timeout_s_ {1.0};
  double feedback_timeout_s_ {5.0};
  double min_altitude_m_ {2.0};
  double geofence_cube_size_m_ {2000.0};
  bool discover_drones_ {true};
  bool require_registration_ {true};
  double heartbeat_period_s_ {0.5};
  double lease_timeout_s_ {3.0};
  std::size_t route_improvement_passes_ {50};
  double home_altitude_above_origin_m_ {15.0};
  double home_cruise_speed_m_s_ {15.0};
  double home_precision_speed_m_s_ {3.0};
  bool home_use_fixed_wing_ {true};
  double min_drone_speed_m_s_ {10.0};
  double max_drone_speed_m_s_ {20.0};
  double min_lidar_range_m_ {70.0};
  double max_lidar_range_m_ {300.0};
  double default_lidar_range_m_ {70.0};
  double battery_low_pct_ {40.0};
  double battery_critical_pct_ {25.0};
  double battery_penalty_max_s_ {120.0};
  double battery_reserve_s_ {60.0};
  double battery_energy_reserve_pct_ {15.0};
  double active_route_penalty_s_ {30.0};
  double routing_vertical_speed_m_s_ {3.0};
  std::unique_ptr<FlightCostModel> flight_cost_model_;
  std::vector<double> no_fly_zone_values_;
  uint64_t next_target_id_ {1};
  uint64_t next_route_token_ {1};
  uint64_t next_command_id_ {1};
  uint64_t next_mission_id_ {1};
  uint64_t active_command_id_ {0};
  uint64_t active_mission_id_ {0};
  uint32_t active_revision_ {0};
  std::vector<std::shared_ptr<DroneRecord>> drones_;
  std::vector<TargetRecord> pending_targets_;
  std::map<uint64_t, TargetRecord> active_targets_;
  std::map<std::string, RouteRecord> current_routes_;
  std::map<std::string, RouteRecord> provisional_routes_;
  std::map<std::string, RouteRecord> planned_routes_;
  std::map<std::string, AuxiliaryRoute> auxiliary_routes_;
  std::vector<SafetyEvent> pending_safety_events_;
  DispatchPhase dispatch_phase_ {DispatchPhase::IDLE};
  bool recalculation_in_progress_ {false};
  bool planned_recalculation_ {false};
  rclcpp::Time dispatch_deadline_;
  std::chrono::steady_clock::time_point last_discovery_;
  std::string status_message_ {"waiting for targets"};
  rclcpp::Publisher<SwarmMissionCommand>::SharedPtr mission_command_pub_;
  rclcpp::Subscription<SwarmMissionFeedback>::SharedPtr mission_feedback_sub_;
  rclcpp::Subscription<SwarmDroneHeartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Service<AddSwarmTarget>::SharedPtr add_target_service_;
  rclcpp::Service<RemoveSwarmTarget>::SharedPtr remove_target_service_;
  rclcpp::Service<SwarmCommand>::SharedPtr command_service_;
  rclcpp::Service<RegisterSwarmDrone>::SharedPtr register_drone_service_;
  rclcpp::Publisher<SwarmState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_swarm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_swarm::SwarmCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
