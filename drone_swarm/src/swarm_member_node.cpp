#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

#include <drone_interfaces/action/navigate_to.hpp>
#include <drone_interfaces/msg/swarm_drone_heartbeat.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <drone_interfaces/srv/register_swarm_drone.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateTo = drone_interfaces::action::NavigateTo;
using RegisterSwarmDrone = drone_interfaces::srv::RegisterSwarmDrone;
using SwarmDroneHeartbeat = drone_interfaces::msg::SwarmDroneHeartbeat;
using VehicleState = drone_interfaces::msg::VehicleState;

namespace drone_swarm
{

class SwarmMemberNode : public rclcpp::Node
{
public:
  SwarmMemberNode()
  : Node("swarm_member"), boot_id_(makeBootId())
  {
    drone_id_ = declare_parameter<std::string>("drone_id", "");
    if (drone_id_.empty()) drone_id_ = droneIdFromNamespace();
    if (drone_id_.empty()) {
      throw std::runtime_error("swarm_member requires drone_id or a non-root namespace");
    }

    state_timeout_s_ = std::max(
      0.1, declare_parameter<double>("state_timeout_sec", 1.0));
    registration_refresh_s_ = std::max(
      0.5, declare_parameter<double>("registration_refresh_sec", 2.0));
    requested_heartbeat_period_s_ = std::max(
      0.1, declare_parameter<double>("heartbeat_period_sec", 0.5));
    heartbeat_period_s_ = requested_heartbeat_period_s_;
    has_lidar_ = declare_parameter<bool>("has_lidar", true);
    supports_fixed_wing_ = declare_parameter<bool>("supports_fixed_wing", true);
    supports_vtol_ = declare_parameter<bool>("supports_vtol", true);

    drone_namespace_ = normalizedNamespace(get_namespace());
    if (drone_namespace_.empty()) drone_namespace_ = "/" + drone_id_;
    state_topic_ = drone_namespace_ + "/vehicle/state";
    navigate_action_ = drone_namespace_ + "/navigate_to";
    arm_service_ = drone_namespace_ + "/flight/arm";
    takeoff_action_ = drone_namespace_ + "/takeoff";

    state_sub_ = create_subscription<VehicleState>(
      "vehicle/state", 10,
      [this](const VehicleState::SharedPtr message) {
        state_ = *message;
        have_state_ = true;
        last_state_update_ = std::chrono::steady_clock::now();
      });
    navigate_client_ = rclcpp_action::create_client<NavigateTo>(this, "navigate_to");
    registration_client_ = create_client<RegisterSwarmDrone>("/swarm/register_drone");
    heartbeat_pub_ = create_publisher<SwarmDroneHeartbeat>(
      "/swarm/drone_heartbeat", rclcpp::QoS(20).reliable());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SwarmMemberNode::onTimer, this));

    const auto current_time = std::chrono::steady_clock::now();
    last_registration_request_ = current_time - std::chrono::seconds(10);
    last_heartbeat_ = current_time - std::chrono::seconds(10);
    RCLCPP_INFO(
      get_logger(), "Swarm member %s started with boot ID %s",
      drone_id_.c_str(), boot_id_.c_str());
  }

private:
  static std::string normalizedNamespace(std::string value)
  {
    if (value.empty() || value == "/") return "";
    if (value.front() != '/') value.insert(value.begin(), '/');
    while (value.size() > 1 && value.back() == '/') value.pop_back();
    return value;
  }

  std::string droneIdFromNamespace() const
  {
    const auto node_namespace = normalizedNamespace(get_namespace());
    if (node_namespace.empty()) return "";
    return node_namespace.substr(node_namespace.find_last_of('/') + 1);
  }

  static std::string makeBootId()
  {
    std::random_device random_device;
    std::mt19937_64 generator(random_device());
    std::uniform_int_distribution<uint64_t> distribution;
    std::ostringstream value;
    value << std::hex << std::setfill('0')
          << std::setw(16) << distribution(generator)
          << std::setw(16) << distribution(generator);
    return value.str();
  }

  bool stateFresh(const std::chrono::steady_clock::time_point & current_time) const
  {
    if (!have_state_) return false;
    return std::chrono::duration<double>(current_time - last_state_update_).count() <=
           state_timeout_s_;
  }

  void onTimer()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const bool px4_ready = stateFresh(current_time);
    const bool localized = px4_ready && state_.position_valid;
    const bool navigation_ready = navigate_client_->action_server_is_ready();

    const bool registration_due =
      std::chrono::duration<double>(current_time - last_registration_request_).count() >=
      registration_refresh_s_;
    if (px4_ready && localized && navigation_ready && registration_due &&
      !registration_request_in_progress_)
    {
      requestRegistration(current_time);
    }
    if (!registered_) {
      const char * reason = !px4_ready ? "waiting for fresh PX4 vehicle state" :
        (!localized ? "waiting for a valid localized position" :
        (!navigation_ready ? "waiting for the navigation action server" :
        (registration_request_in_progress_ ? "registration request in progress" :
        "waiting to send registration request")));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Not ready to join swarm: %s", reason);
    }

    const bool heartbeat_due =
      std::chrono::duration<double>(current_time - last_heartbeat_).count() >=
      heartbeat_period_s_;
    if (registered_ && heartbeat_due) {
      publishHeartbeat(current_time, px4_ready, localized, navigation_ready);
    }
  }

  void requestRegistration(const std::chrono::steady_clock::time_point & current_time)
  {
    last_registration_request_ = current_time;
    if (!registration_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for /swarm/register_drone");
      return;
    }

    auto request = std::make_shared<RegisterSwarmDrone::Request>();
    request->drone_id = drone_id_;
    request->drone_namespace = drone_namespace_;
    request->boot_id = boot_id_;
    request->state_topic = state_topic_;
    request->navigate_action = navigate_action_;
    request->arm_service = arm_service_;
    request->takeoff_action = takeoff_action_;
    request->has_lidar = has_lidar_;
    request->supports_fixed_wing = supports_fixed_wing_;
    request->supports_vtol = supports_vtol_;
    registration_request_in_progress_ = true;
    registration_client_->async_send_request(
      request,
      [this](rclcpp::Client<RegisterSwarmDrone>::SharedFuture future) {
        registration_request_in_progress_ = false;
        const auto response = future.get();
        registered_ = response->accepted;
        if (response->accepted) {
          heartbeat_period_s_ = std::max(0.1, response->heartbeat_period_sec);
          if (!registration_announced_) {
            RCLCPP_INFO(
              get_logger(), "Joined swarm: %s (lease %.1fs)",
              response->message.c_str(), response->lease_timeout_sec);
            registration_announced_ = true;
          }
        } else {
          registration_announced_ = false;
          RCLCPP_WARN(get_logger(), "Swarm registration rejected: %s", response->message.c_str());
        }
      });
  }

  void publishHeartbeat(
    const std::chrono::steady_clock::time_point & current_time,
    bool px4_ready,
    bool localized,
    bool navigation_ready)
  {
    SwarmDroneHeartbeat heartbeat;
    heartbeat.header.stamp = now();
    heartbeat.header.frame_id = "map";
    heartbeat.drone_id = drone_id_;
    heartbeat.boot_id = boot_id_;
    heartbeat.px4_ready = px4_ready;
    heartbeat.localized = localized;
    heartbeat.navigation_ready = navigation_ready;
    heartbeat.lidar_ready = has_lidar_;
    heartbeat_pub_->publish(heartbeat);
    last_heartbeat_ = current_time;
  }

  std::string drone_id_;
  std::string drone_namespace_;
  std::string boot_id_;
  std::string state_topic_;
  std::string navigate_action_;
  std::string arm_service_;
  std::string takeoff_action_;
  double state_timeout_s_ {1.0};
  double registration_refresh_s_ {2.0};
  double requested_heartbeat_period_s_ {0.5};
  double heartbeat_period_s_ {0.5};
  bool has_lidar_ {true};
  bool supports_fixed_wing_ {true};
  bool supports_vtol_ {true};
  bool have_state_ {false};
  bool registered_ {false};
  bool registration_request_in_progress_ {false};
  bool registration_announced_ {false};
  VehicleState state_;
  std::chrono::steady_clock::time_point last_state_update_;
  std::chrono::steady_clock::time_point last_registration_request_;
  std::chrono::steady_clock::time_point last_heartbeat_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp_action::Client<NavigateTo>::SharedPtr navigate_client_;
  rclcpp::Client<RegisterSwarmDrone>::SharedPtr registration_client_;
  rclcpp::Publisher<SwarmDroneHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_swarm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_swarm::SwarmMemberNode>());
  rclcpp::shutdown();
  return 0;
}
