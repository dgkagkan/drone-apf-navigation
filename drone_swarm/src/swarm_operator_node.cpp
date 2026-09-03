#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <drone_interfaces/action/takeoff.hpp>
#include <drone_interfaces/msg/swarm_drone_state.hpp>
#include <drone_interfaces/msg/swarm_state.hpp>
#include <drone_interfaces/srv/arm.hpp>
#include <drone_interfaces/srv/add_swarm_target.hpp>
#include <drone_interfaces/srv/swarm_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using AddSwarmTarget = drone_interfaces::srv::AddSwarmTarget;
using Arm = drone_interfaces::srv::Arm;
using SwarmCommand = drone_interfaces::srv::SwarmCommand;
using SwarmDroneState = drone_interfaces::msg::SwarmDroneState;
using SwarmState = drone_interfaces::msg::SwarmState;
using Takeoff = drone_interfaces::action::Takeoff;
using TakeoffGoalHandle = rclcpp_action::ClientGoalHandle<Takeoff>;
using std::placeholders::_1;

class SwarmOperatorNode : public rclcpp::Node
{
public:
  SwarmOperatorNode()
  : Node("swarm_operator")
  {
    cruise_speed_m_s_ = declare_parameter<double>("cruise_speed_m_s", 15.0);
    use_fixed_wing_ = declare_parameter<bool>("use_fixed_wing", true);
    takeoff_altitude_m_ = declare_parameter<double>("takeoff_altitude_m", 15.0);
    takeoff_climb_speed_m_s_ = declare_parameter<double>("takeoff_climb_speed_m_s", 2.0);
    configureDrones();
    add_target_client_ = create_client<AddSwarmTarget>("/swarm/add_target");
    command_client_ = create_client<SwarmCommand>("/swarm/command");
    state_sub_ = create_subscription<SwarmState>(
      "/swarm/state", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&SwarmOperatorNode::onState, this, _1));
    running_ = true;
    input_thread_ = std::thread(&SwarmOperatorNode::inputLoop, this);
  }

  ~SwarmOperatorNode() override
  {
    running_ = false;
    if (input_thread_.joinable()) input_thread_.join();
  }

private:
  struct DroneControl
  {
    std::string id;
    std::string drone_namespace;
    rclcpp::Client<Arm>::SharedPtr arm_client;
    rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client;
    TakeoffGoalHandle::SharedPtr takeoff_goal_handle;
    bool takeoff_in_progress {false};
    std::string takeoff_phase;
    bool registered {false};
    bool connected {false};
    bool available {false};
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
      "drone_ids", std::vector<std::string>{"drone_1", "drone_2", "drone_3"});
    auto drone_namespaces = declare_parameter<std::vector<std::string>>(
      "drone_namespaces", std::vector<std::string>{});
    if (!drone_namespaces.empty() && drone_namespaces.size() != drone_ids.size()) {
      throw std::runtime_error("drone_namespaces must be empty or match drone_ids");
    }
    for (std::size_t index = 0; index < drone_ids.size(); ++index) {
      const auto drone_namespace = normalizeNamespace(
        drone_namespaces.empty() ? drone_ids[index] : drone_namespaces[index]);
      addOrUpdateDrone(drone_ids[index], drone_namespace);
    }
  }

  std::shared_ptr<DroneControl> addOrUpdateDrone(
    const std::string & drone_id,
    const std::string & drone_namespace)
  {
    std::lock_guard<std::mutex> lock(drone_mutex_);
    const auto found = std::find_if(
      drones_.begin(), drones_.end(),
      [&drone_id](const auto & drone) {return drone->id == drone_id;});
    if (found != drones_.end()) {
      if ((*found)->drone_namespace != drone_namespace) {
        (*found)->drone_namespace = drone_namespace;
        (*found)->arm_client = create_client<Arm>(drone_namespace + "/flight/arm");
        (*found)->takeoff_client = rclcpp_action::create_client<Takeoff>(
          this, drone_namespace + "/takeoff");
      }
      return *found;
    }

    auto drone = std::make_shared<DroneControl>();
    drone->id = drone_id;
    drone->drone_namespace = drone_namespace;
    drone->arm_client = create_client<Arm>(drone_namespace + "/flight/arm");
    drone->takeoff_client = rclcpp_action::create_client<Takeoff>(
      this, drone_namespace + "/takeoff");
    drones_.push_back(drone);
    RCLCPP_INFO(
      get_logger(), "Operator discovered %s in namespace %s",
      drone_id.c_str(), drone_namespace.c_str());
    return drone;
  }

  void onState(const SwarmState::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = *message;
      have_state_ = true;
    }
    for (const auto & state : message->drones) {
      const auto drone = addOrUpdateDrone(state.drone_id, state.drone_namespace);
      std::lock_guard<std::mutex> lock(drone_mutex_);
      drone->registered = state.registered;
      drone->connected = state.connected;
      drone->available = state.available;
    }
  }

  void inputLoop()
  {
    printLine("Swarm operator ready. CALCULATE previews routes; START dispatches them.");
    printHelp();
    printPrompt();
    std::string input_buffer;
    while (running_ && rclcpp::ok()) {
      pollfd input_poll{STDIN_FILENO, POLLIN, 0};
      const int poll_result = poll(&input_poll, 1, 200);
      if (poll_result <= 0) continue;
      if (!(input_poll.revents & POLLIN)) break;
      char input_chunk[512];
      const ssize_t bytes_read = read(STDIN_FILENO, input_chunk, sizeof(input_chunk));
      if (bytes_read <= 0) break;
      input_buffer.append(input_chunk, static_cast<std::size_t>(bytes_read));
      std::size_t newline = 0;
      while ((newline = input_buffer.find('\n')) != std::string::npos) {
        std::string line = input_buffer.substr(0, newline);
        input_buffer.erase(0, newline + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        processInput(line);
        if (running_ && rclcpp::ok()) printPrompt();
      }
    }
    running_ = false;
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  void processInput(const std::string & line)
  {
    std::istringstream input(line);
    std::string command;
    input >> command;
    if (command.empty()) return;

    if (command == "arm") {
      handleArmCommand(input);
    } else if (command == "takeoff") {
      handleTakeoffCommand(input);
    } else if (command == "goal" || command == "add_target") {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      std::string trailing;
      if (!(input >> x >> y >> z) || input >> trailing) {
        printLine("expected: goal <x> <y> <z>");
        return;
      }
      addTarget(x, y, z);
    } else if (command == "calculate") {
      sendCommand(SwarmCommand::Request::CALCULATE, "CALCULATE");
    } else if (command == "recalculate") {
      sendCommand(SwarmCommand::Request::RECALCULATE, "RECALCULATE");
    } else if (command == "start" || command == "go") {
      sendCommand(SwarmCommand::Request::START_MISSION, "START_MISSION");
    } else if (command == "clear") {
      sendCommand(SwarmCommand::Request::CLEAR_PENDING_TARGETS, "CLEAR_PENDING_TARGETS");
    } else if (command == "cancel") {
      sendCommand(SwarmCommand::Request::CANCEL_ACTIVE_MISSION, "CANCEL_ACTIVE_MISSION");
    } else if (command == "home") {
      sendCommand(SwarmCommand::Request::RETURN_HOME, "RETURN_HOME");
    } else if (command == "status") {
      printStatus();
    } else if (command == "help") {
      printHelp();
    } else if (command == "quit" || command == "exit") {
      running_ = false;
      rclcpp::shutdown();
    } else {
      printLine("unknown command: " + command);
    }
  }

  std::vector<std::shared_ptr<DroneControl>> selectDrones(
    const std::string & selector, std::string & error) const
  {
    std::lock_guard<std::mutex> lock(drone_mutex_);
    if (selector.empty()) {
      std::vector<std::shared_ptr<DroneControl>> connected;
      std::copy_if(
        drones_.begin(), drones_.end(), std::back_inserter(connected),
        [](const auto & drone) {return drone->connected;});
      if (connected.empty()) error = "no connected swarm drones";
      return connected;
    }
    std::string drone_id = selector;
    if (selector.find_first_not_of("0123456789") == std::string::npos) {
      drone_id = "drone_" + selector;
    }
    const auto found = std::find_if(
      drones_.begin(), drones_.end(),
      [&drone_id](const auto & drone) {return drone->id == drone_id;});
    if (found == drones_.end()) {
      error = "unknown drone selector: " + selector;
      return {};
    }
    return {*found};
  }

  void handleArmCommand(std::istringstream & input)
  {
    std::string selector;
    std::string trailing;
    input >> selector;
    if (input >> trailing) {
      printLine("expected: arm [drone_number]");
      return;
    }
    std::string error;
    const auto selected = selectDrones(selector, error);
    if (!error.empty()) {
      printLine("ARM rejected: " + error);
      return;
    }
    sendCommand(
      SwarmCommand::Request::ARM, "ARM",
      selected.size() == 1 ? selected.front()->id : "");
  }

  void requestArm(const std::vector<std::shared_ptr<DroneControl>> & selected)
  {
    const auto unavailable = std::find_if(
      selected.begin(), selected.end(),
      [](const auto & drone) {return !drone->arm_client->service_is_ready();});
    if (unavailable != selected.end()) {
      printLine("ARM rejected: service unavailable for " + (*unavailable)->id);
      return;
    }
    for (const auto & drone : selected) {
      auto request = std::make_shared<Arm::Request>();
      request->arm = true;
      drone->arm_client->async_send_request(
        request,
        [this, drone](rclcpp::Client<Arm>::SharedFuture future) {
          const auto response = future.get();
          printLine(
            drone->id + (response->accepted ? " ARM accepted: " : " ARM rejected: ") +
            response->message);
        });
      printLine("ARM request sent to " + drone->id);
    }
  }

  void handleTakeoffCommand(std::istringstream & input)
  {
    std::string selector;
    std::string trailing;
    input >> selector;
    if (input >> trailing) {
      printLine("expected: takeoff [drone_number]");
      return;
    }
    std::string error;
    const auto selected = selectDrones(selector, error);
    if (!error.empty()) {
      printLine("TAKEOFF rejected: " + error);
      return;
    }
    sendCommand(
      SwarmCommand::Request::TAKEOFF, "TAKEOFF",
      selected.size() == 1 ? selected.front()->id : "",
      takeoff_altitude_m_, takeoff_climb_speed_m_s_);
  }

  void requestTakeoff(const std::vector<std::shared_ptr<DroneControl>> & selected)
  {
    const auto unavailable = std::find_if(
      selected.begin(), selected.end(),
      [](const auto & drone) {return !drone->takeoff_client->action_server_is_ready();});
    if (unavailable != selected.end()) {
      printLine("TAKEOFF rejected: action server unavailable for " + (*unavailable)->id);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(flight_mutex_);
      const auto active = std::find_if(
        selected.begin(), selected.end(),
        [](const auto & drone) {return drone->takeoff_in_progress;});
      if (active != selected.end()) {
        printLine("TAKEOFF rejected: already active for " + (*active)->id);
        return;
      }
      for (const auto & drone : selected) {
        drone->takeoff_in_progress = true;
        drone->takeoff_phase.clear();
      }
    }

    for (const auto & drone : selected) {
      Takeoff::Goal goal;
      goal.target_altitude_m = takeoff_altitude_m_;
      goal.climb_speed_m_s = takeoff_climb_speed_m_s_;
      rclcpp_action::Client<Takeoff>::SendGoalOptions options;
      options.goal_response_callback =
        [this, drone](const TakeoffGoalHandle::SharedPtr & goal_handle) {
          {
            std::lock_guard<std::mutex> lock(flight_mutex_);
            drone->takeoff_goal_handle = goal_handle;
            if (!goal_handle) drone->takeoff_in_progress = false;
          }
          printLine(
            drone->id + (goal_handle ? " TAKEOFF accepted" : " TAKEOFF rejected"));
        };
      options.feedback_callback =
        [this, drone](TakeoffGoalHandle::SharedPtr,
        const std::shared_ptr<const Takeoff::Feedback> feedback) {
          bool phase_changed = false;
          {
            std::lock_guard<std::mutex> lock(flight_mutex_);
            phase_changed = drone->takeoff_phase != feedback->phase;
            drone->takeoff_phase = feedback->phase;
          }
          if (phase_changed) {
            printLine(drone->id + " TAKEOFF: " + feedback->phase);
          }
        };
      options.result_callback =
        [this, drone](const TakeoffGoalHandle::WrappedResult & result) {
          const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success;
          const std::string message = result.result ? result.result->message : "no result";
          {
            std::lock_guard<std::mutex> lock(flight_mutex_);
            drone->takeoff_in_progress = false;
            drone->takeoff_goal_handle.reset();
            drone->takeoff_phase.clear();
          }
          printLine(
            drone->id + (succeeded ? " TAKEOFF complete: " : " TAKEOFF failed: ") + message);
        };
      drone->takeoff_client->async_send_goal(goal, options);
      printLine(
        "TAKEOFF request sent to " + drone->id + " (altitude=" +
        std::to_string(takeoff_altitude_m_) + "m)");
    }
  }

  void addTarget(double x, double y, double z)
  {
    if (!add_target_client_->service_is_ready()) {
      printLine("ADD_TARGET failed: /swarm/add_target is unavailable");
      return;
    }
    auto request = std::make_shared<AddSwarmTarget::Request>();
    request->target.header.frame_id = "map";
    request->target.pose.position.x = x;
    request->target.pose.position.y = y;
    request->target.pose.position.z = z;
    request->target.pose.orientation.w = 1.0;
    request->cruise_speed_m_s = cruise_speed_m_s_;
    request->use_fixed_wing = use_fixed_wing_;
    add_target_client_->async_send_request(
      request,
      [this](rclcpp::Client<AddSwarmTarget>::SharedFuture future) {
        const auto response = future.get();
        std::ostringstream output;
        output << (response->accepted ? "ADD_TARGET accepted" : "ADD_TARGET rejected")
               << ": " << response->message;
        if (response->accepted) output << " (target " << response->target_id << ')';
        printLine(output.str());
      });
  }

  void sendCommand(
    uint8_t command,
    const std::string & name,
    const std::string & drone_id = "",
    double takeoff_altitude_m = 0.0,
    double takeoff_climb_speed_m_s = 0.0)
  {
    if (!command_client_->service_is_ready()) {
      printLine(name + " failed: /swarm/command is unavailable");
      return;
    }
    auto request = std::make_shared<SwarmCommand::Request>();
    request->command = command;
    request->drone_id = drone_id;
    request->takeoff_altitude_m = takeoff_altitude_m;
    request->takeoff_climb_speed_m_s = takeoff_climb_speed_m_s;
    command_client_->async_send_request(
      request,
      [this, name](rclcpp::Client<SwarmCommand>::SharedFuture future) {
        const auto response = future.get();
        printLine(
          name + (response->accepted ? " accepted: " : " rejected: ") + response->message);
      });
  }

  void printStatus()
  {
    SwarmState state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_state_) {
        printLine("swarm state is unavailable");
        return;
      }
      state = state_;
    }
    std::ostringstream output;
    output << "pending=" << state.pending_target_count
           << " active=" << state.active_target_count
           << " available_drones=" << state.available_drone_count
           << " assignments=" << state.assignments.size()
           << " planned_assignments=" << state.planned_assignments.size()
           << " | " << state.status_message;
    printLine(output.str());
    for (const auto & assignment : state.assignments) {
      std::ostringstream line;
      line << "  " << assignment.drone_id << " -> target " << assignment.target_id
           << " cost=" << assignment.cost << " | " << assignment.message;
      printLine(line.str());
    }
    for (const auto & assignment : state.planned_assignments) {
      std::ostringstream line;
      line << "  PREVIEW " << assignment.drone_id << " -> target "
           << assignment.target_id << " cost=" << assignment.cost;
      printLine(line.str());
    }
    for (const auto & drone : state.drones) {
      std::ostringstream line;
      line << "  " << drone.drone_id
           << " connected=" << std::boolalpha << drone.connected
           << " localized=" << drone.localized
           << " available=" << drone.available
           << " busy=" << drone.busy
           << " armed=" << drone.armed
           << " position=(" << drone.position.x << ", "
           << drone.position.y << ", " << drone.position.z << ')';
      printLine(line.str());
    }
  }

  void printHelp()
  {
    printLine(
      "commands: arm [drone_number] | takeoff [drone_number] | "
      "goal <x> <y> <z> | calculate | recalculate | start (or go) | clear | "
      "cancel | home | status | help | quit");
  }

  void printLine(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << "\r\033[2K" << line << '\n';
    if (running_ && rclcpp::ok()) std::cout << "swarm> " << std::flush;
  }

  void printPrompt()
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << "swarm> " << std::flush;
  }

  std::atomic<bool> running_ {false};
  double cruise_speed_m_s_ {15.0};
  bool use_fixed_wing_ {true};
  double takeoff_altitude_m_ {15.0};
  double takeoff_climb_speed_m_s_ {2.0};
  std::thread input_thread_;
  std::mutex output_mutex_;
  std::mutex state_mutex_;
  std::mutex flight_mutex_;
  mutable std::mutex drone_mutex_;
  bool have_state_ {false};
  SwarmState state_;
  std::vector<std::shared_ptr<DroneControl>> drones_;
  rclcpp::Client<AddSwarmTarget>::SharedPtr add_target_client_;
  rclcpp::Client<SwarmCommand>::SharedPtr command_client_;
  rclcpp::Subscription<SwarmState>::SharedPtr state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwarmOperatorNode>());
  rclcpp::shutdown();
  return 0;
}
