#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <drone_interfaces/action/navigate_to.hpp>
#include <drone_interfaces/action/takeoff.hpp>
#include <drone_interfaces/msg/apf_telemetry.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <drone_interfaces/srv/arm.hpp>
#include <drone_interfaces/srv/set_apf_mode.hpp>
#include <drone_interfaces/srv/validate_goal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

using Arm = drone_interfaces::srv::Arm;
using ApfTelemetry = drone_interfaces::msg::ApfTelemetry;
using NavigateTo = drone_interfaces::action::NavigateTo;
using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateTo>;
using SetApfMode = drone_interfaces::srv::SetApfMode;
using SetBool = std_srvs::srv::SetBool;
using Takeoff = drone_interfaces::action::Takeoff;
using TakeoffGoalHandle = rclcpp_action::ClientGoalHandle<Takeoff>;
using ValidateGoal = drone_interfaces::srv::ValidateGoal;
using VehicleState = drone_interfaces::msg::VehicleState;
using std::placeholders::_1;

class NavigationClientNode : public rclcpp::Node
{
public:
  NavigationClientNode()
  : Node("navigation_client")
  {
    cruise_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("cruise_speed_m_s", 20.0));
    feedback_period_s_ = std::max(
      0.1, declare_parameter<double>("feedback_period_s", 0.5));
    takeoff_altitude_m_ = std::max(
      0.6, declare_parameter<double>("takeoff_altitude_m", 15.0));
    takeoff_climb_speed_m_s_ = std::max(
      0.2, declare_parameter<double>("takeoff_climb_speed_m_s", 2.0));

    action_client_ = rclcpp_action::create_client<NavigateTo>(this, "/navigate_to");
    takeoff_client_ = rclcpp_action::create_client<Takeoff>(this, "/takeoff");
    validation_client_ = create_client<ValidateGoal>("/navigation/validate_goal");
    arm_client_ = create_client<Arm>("/flight/arm");
    apf_enabled_client_ = create_client<SetBool>("/apf/set_enabled");
    apf_mode_client_ = create_client<SetApfMode>("/apf/set_mode");
    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&NavigationClientNode::onState, this, _1));
    apf_sub_ = create_subscription<ApfTelemetry>(
      "/apf/telemetry", rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&NavigationClientNode::onApfTelemetry, this, _1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&NavigationClientNode::onTimer, this));

    running_ = true;
    input_thread_ = std::thread(&NavigationClientNode::inputLoop, this);
  }

  ~NavigationClientNode() override
  {
    running_ = false;
    if (input_thread_.joinable()) input_thread_.join();
  }

private:
  struct QueuedGoal
  {
    uint64_t id {0};
    NavigateTo::Goal goal;
  };

  void onState(const VehicleState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    state_ = *message;
    have_state_ = true;
  }

  void onApfTelemetry(const ApfTelemetry::SharedPtr message)
  {
    if (message->active_mode.empty()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    active_apf_mode_ = message->active_mode;
  }

  void printLine(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (interactive_terminal_) {
      recent_messages_.push_back(line);
      if (recent_messages_.size() > max_recent_messages_) recent_messages_.pop_front();
      renderTerminalLocked();
      return;
    }
    std::cout << "\r\033[2K" << line << '\n';
    if (running_ && rclcpp::ok()) {
      std::cout << promptText() << std::flush;
    }
  }

  std::string promptText() const
  {
    return awaiting_coordinates_ ? "give coordinates (x y z)> " : "give command> ";
  }

  void printPrompt()
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (interactive_terminal_) {
      renderTerminalLocked();
      return;
    }
    std::cout << promptText() << std::flush;
  }

  std::string helpText() const
  {
    return
      "arm | takeoff [altitude] [climb_speed] | goal [x y z] | speed <m/s> | "
      "apf <on|off> | apf mode <stable|normal|sport> | status | queue | "
      "cancel | clear | help | quit";
  }

  void printHelp()
  {
    printLine("commands: " + helpText());
  }

  void setLiveFeedback(const std::string & feedback)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    live_feedback_ = feedback;
    if (interactive_terminal_) {
      renderTerminalLocked();
    } else {
      std::cout << "\r\033[2K" << feedback << '\n';
      if (running_ && rclcpp::ok()) std::cout << promptText() << std::flush;
    }
  }

  void renderTerminalLocked()
  {
    std::cout << "\033[H\033[2J"
              << "Drone navigation client\n"
              << "Goals are validated, queued, and executed in FIFO order.\n"
              << "APF safety and its flight profile can be changed live.\n\n"
              << "COMMANDS\n  " << helpText() << "\n\n"
              << "LIVE FEEDBACK\n"
              << (live_feedback_.empty() ? "  Waiting for an active operation..." : live_feedback_)
              << "\n\nRECENT EVENTS\n";
    if (recent_messages_.empty()) {
      std::cout << "  No events yet.\n";
    } else {
      for (const auto & message : recent_messages_) std::cout << "  " << message << '\n';
    }
    std::cout << "\n" << promptText() << input_buffer_ << std::flush;
  }

  bool enableInteractiveTerminal()
  {
    if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) return false;
    if (tcgetattr(STDIN_FILENO, &original_terminal_) != 0) return false;

    termios terminal = original_terminal_;
    terminal.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    terminal.c_cc[VMIN] = 0;
    terminal.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &terminal) != 0) return false;

    terminal_mode_active_ = true;
    std::cout << "\033[?1049h\033[?25h" << std::flush;
    return true;
  }

  void restoreTerminal()
  {
    if (!terminal_mode_active_) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
    terminal_mode_active_ = false;
    std::cout << "\033[?1049l" << std::flush;
  }

  void inputLoop()
  {
    interactive_terminal_ = enableInteractiveTerminal();
    if (interactive_terminal_) {
      std::lock_guard<std::mutex> lock(output_mutex_);
      recent_messages_.push_back("client ready; enter arm to begin");
      renderTerminalLocked();
    } else {
      std::lock_guard<std::mutex> lock(output_mutex_);
      std::cout << "\nDrone navigation client\n"
                << "Goals are validated, queued, and executed in FIFO order.\n"
                << "Every goal uses fixed-wing fast mode and the shared 3D APF.\n\n";
    }
    if (!interactive_terminal_) printHelp();

    while (running_ && rclcpp::ok()) {
      pollfd input_poll{STDIN_FILENO, POLLIN, 0};
      const int poll_result = poll(&input_poll, 1, 200);
      if (poll_result <= 0) continue;
      if (!(input_poll.revents & POLLIN)) {
        if (input_poll.revents & (POLLHUP | POLLERR | POLLNVAL)) {
          running_ = false;
          rclcpp::shutdown();
        }
        continue;
      }

      char input_chunk[512];
      const ssize_t bytes_read = read(STDIN_FILENO, input_chunk, sizeof(input_chunk));
      if (bytes_read <= 0) {
        running_ = false;
        rclcpp::shutdown();
        break;
      }
      if (interactive_terminal_) {
        processInteractiveCharacters(input_chunk, static_cast<std::size_t>(bytes_read));
      } else {
        processBufferedInput(input_chunk, static_cast<std::size_t>(bytes_read));
      }
    }
    restoreTerminal();
  }

  void processBufferedInput(const char * characters, std::size_t count)
  {
    input_buffer_.append(characters, count);
    std::size_t newline = 0;
    while ((newline = input_buffer_.find('\n')) != std::string::npos) {
      std::string line = input_buffer_.substr(0, newline);
      input_buffer_.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      processInput(line);
      if (!running_ || !rclcpp::ok()) break;
    }
  }

  void processInteractiveCharacters(const char * characters, std::size_t count)
  {
    std::deque<std::string> completed_lines;
    bool shutdown_requested = false;
    {
      std::lock_guard<std::mutex> lock(output_mutex_);
      for (std::size_t index = 0; index < count; ++index) {
        const unsigned char character = static_cast<unsigned char>(characters[index]);
        if (consumeEscapeSequence(character)) continue;
        if (character == 3 || (character == 4 && input_buffer_.empty())) {
          shutdown_requested = true;
          break;
        }
        if (character == '\r' || character == '\n') {
          completed_lines.push_back(input_buffer_);
          input_buffer_.clear();
        } else if (character == 8 || character == 127) {
          if (!input_buffer_.empty()) input_buffer_.pop_back();
        } else if (std::isprint(character)) {
          input_buffer_.push_back(static_cast<char>(character));
        }
      }
      renderTerminalLocked();
    }

    if (shutdown_requested) {
      running_ = false;
      rclcpp::shutdown();
      return;
    }
    for (const auto & line : completed_lines) {
      processInput(line);
      if (!running_ || !rclcpp::ok()) break;
    }
  }

  bool consumeEscapeSequence(unsigned char character)
  {
    if (character == 27) {
      escape_sequence_state_ = 1;
      return true;
    }
    if (escape_sequence_state_ == 1) {
      escape_sequence_state_ = character == '[' ? 2 : 0;
      return true;
    }
    if (escape_sequence_state_ == 2) {
      if (character >= 0x40 && character <= 0x7e) escape_sequence_state_ = 0;
      return true;
    }
    return false;
  }

  void processInput(const std::string & line)
  {
    if (awaiting_coordinates_) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      std::istringstream coordinates(line);
      if (!(coordinates >> x >> y >> z)) {
        printLine("invalid coordinates; expected: x y z");
        return;
      }
      awaiting_coordinates_ = false;
      proposeGoal(x, y, z);
      return;
    }

    std::istringstream command_stream(line);
    std::string command;
    command_stream >> command;
    if (command.empty()) {
      printPrompt();
      return;
    }

    if (command == "arm") {
      requestArm();
    } else if (command == "takeoff") {
      double altitude_m = takeoff_altitude_m_;
      double climb_speed_m_s = takeoff_climb_speed_m_s_;
      std::string argument;
      if (command_stream >> argument && !parseNumber(argument, altitude_m)) {
        printLine("invalid takeoff altitude: " + argument);
        return;
      }
      if (command_stream >> argument && !parseNumber(argument, climb_speed_m_s)) {
        printLine("invalid takeoff climb speed: " + argument);
        return;
      }
      if (command_stream >> argument) {
        printLine("invalid takeoff; expected: takeoff [altitude] [climb_speed]");
        return;
      }
      requestTakeoff(altitude_m, climb_speed_m_s);
    } else if (command == "goal" || command == "g") {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (command_stream >> x >> y >> z) {
        proposeGoal(x, y, z);
      } else {
        awaiting_coordinates_ = true;
        printPrompt();
      }
    } else if (command == "speed") {
      double speed = 0.0;
      if (!(command_stream >> speed) || speed <= 0.0) {
        printLine("invalid speed; expected: speed <positive m/s>");
      } else {
        cruise_speed_m_s_ = speed;
        printLine("future goals will use " + formatNumber(speed) + " m/s");
      }
    } else if (command == "apf") {
      std::string operation;
      std::string trailing;
      if (!(command_stream >> operation)) {
        printLine(
          "invalid APF command; expected: apf on | apf off | "
          "apf mode <stable|normal|sport>");
      } else if (operation == "on" || operation == "off") {
        if (command_stream >> trailing) {
          printLine("invalid APF command; expected: apf on | apf off");
        } else {
          requestApfEnabled(operation == "on");
        }
      } else if (operation == "mode") {
        std::string mode;
        if (!(command_stream >> mode) || command_stream >> trailing) {
          printLine("invalid APF mode; expected: apf mode <stable|normal|sport>");
        } else {
          requestApfMode(mode);
        }
      } else {
        printLine(
          "invalid APF command; expected: apf on | apf off | "
          "apf mode <stable|normal|sport>");
      }
    } else if (command == "status") {
      printStatus();
    } else if (command == "queue") {
      printQueue();
    } else if (command == "cancel") {
      cancelActiveGoal();
    } else if (command == "clear") {
      clearWaitingGoals();
    } else if (command == "help") {
      printHelp();
    } else if (command == "quit" || command == "exit") {
      running_ = false;
      rclcpp::shutdown();
    } else {
      printLine("unknown command: " + command);
      printHelp();
    }
  }

  static std::string formatNumber(double value, int precision = 1)
  {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
  }

  static bool parseNumber(const std::string & text, double & value)
  {
    std::istringstream input(text);
    char trailing = '\0';
    return static_cast<bool>(input >> value) && !(input >> trailing);
  }

  void proposeGoal(double x, double y, double z)
  {
    QueuedGoal queued;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      queued.id = next_goal_id_++;
      queued.goal.target.header.frame_id = "map";
      queued.goal.target.pose.position.x = x;
      queued.goal.target.pose.position.y = y;
      queued.goal.target.pose.position.z = z;
      queued.goal.target.pose.orientation.w = 1.0;
      queued.goal.cruise_speed_m_s = cruise_speed_m_s_;
      queued.goal.use_fixed_wing = true;
      pending_validation_.push_back(queued);
    }
    printLine(
      "goal #" + std::to_string(queued.id) + " submitted for validation: (" +
      formatNumber(x) + ", " + formatNumber(y) + ", " + formatNumber(z) + ")");
  }

  void requestArm()
  {
    if (!arm_client_->service_is_ready()) {
      printLine("ARM REJECTED: /flight/arm service is not ready");
      return;
    }
    auto request = std::make_shared<Arm::Request>();
    request->arm = true;
    arm_client_->async_send_request(
      request,
      [this](rclcpp::Client<Arm>::SharedFuture future) {
        const auto response = future.get();
        printLine(
          std::string(response->accepted ? "ARM APPROVED: " : "ARM REJECTED: ") +
          response->message);
      });
    printLine("arm request sent");
  }

  void requestApfEnabled(bool enabled)
  {
    if (!apf_enabled_client_->service_is_ready()) {
      printLine("APF COMMAND REJECTED: /apf/set_enabled service is not ready");
      return;
    }
    auto request = std::make_shared<SetBool::Request>();
    request->data = enabled;
    apf_enabled_client_->async_send_request(
      request,
      [this](rclcpp::Client<SetBool>::SharedFuture future) {
        const auto response = future.get();
        const std::string prefix = response->success ?
          "APF COMMAND APPROVED: " : "APF COMMAND REJECTED: ";
        printLine(prefix + response->message);
      });
    printLine(std::string("APF ") + (enabled ? "ON" : "OFF") + " request sent");
  }

  void requestApfMode(const std::string & mode)
  {
    if (mode != "stable" && mode != "normal" && mode != "sport") {
      printLine("APF MODE REJECTED: expected stable, normal, or sport");
      return;
    }
    if (!apf_mode_client_->service_is_ready()) {
      printLine("APF MODE REJECTED: /apf/set_mode service is not ready");
      return;
    }

    auto request = std::make_shared<SetApfMode::Request>();
    request->mode = mode;
    apf_mode_client_->async_send_request(
      request,
      [this](rclcpp::Client<SetApfMode>::SharedFuture future) {
        const auto response = future.get();
        if (!response->active_mode.empty()) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          active_apf_mode_ = response->active_mode;
        }
        printLine(
          std::string(response->accepted ? "APF MODE APPROVED: " : "APF MODE DECLINED: ") +
          response->message);
      });
    printLine("APF mode request sent: " + mode);
  }

  void requestTakeoff(double altitude_m, double climb_speed_m_s)
  {
    if (!std::isfinite(altitude_m) || altitude_m <= 0.5 ||
      !std::isfinite(climb_speed_m_s) || climb_speed_m_s <= 0.0)
    {
      printLine("invalid takeoff; expected: takeoff <altitude> [positive climb_speed]");
      return;
    }
    if (!takeoff_client_->action_server_is_ready()) {
      printLine("TAKEOFF REJECTED: /takeoff action server is not ready");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (takeoff_in_progress_) {
        printLine("TAKEOFF REJECTED: a takeoff is already active");
        return;
      }
      takeoff_in_progress_ = true;
    }

    Takeoff::Goal goal;
    goal.target_altitude_m = altitude_m;
    goal.climb_speed_m_s = climb_speed_m_s;
    rclcpp_action::Client<Takeoff>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const TakeoffGoalHandle::SharedPtr & goal_handle) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          takeoff_goal_handle_ = goal_handle;
          if (!goal_handle) takeoff_in_progress_ = false;
        }
        printLine(goal_handle ? "TAKEOFF STARTED in MC mode" :
          "TAKEOFF REJECTED by flight supervisor");
      };
    options.feedback_callback =
      [this](TakeoffGoalHandle::SharedPtr,
      const std::shared_ptr<const Takeoff::Feedback> feedback)
      {
        std::ostringstream status;
        status << std::fixed << std::setprecision(1)
               << "  TAKEOFF | " << feedback->phase << '\n'
               << "  altitude=" << feedback->current_altitude_m << "m"
               << " | target=" << takeoff_target_altitude_m_.load() << "m";
        setLiveFeedback(status.str());
      };
    options.result_callback =
      [this](const TakeoffGoalHandle::WrappedResult & result) {
        const std::string message = result.result ? result.result->message : "no result";
        const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          takeoff_in_progress_ = false;
          takeoff_goal_handle_.reset();
        }
        setLiveFeedback(succeeded ? "  Takeoff complete; ready for queued navigation goals." :
          "  Takeoff is not active.");
        printLine(std::string(succeeded ? "TAKEOFF SUCCEEDED: " : "TAKEOFF FAILED: ") + message);
      };

    takeoff_target_altitude_m_ = altitude_m;
    takeoff_client_->async_send_goal(goal, options);
    printLine(
      "takeoff requested: altitude=" + formatNumber(altitude_m) +
      "m, climb=" + formatNumber(climb_speed_m_s) + "m/s");
  }

  void processValidationQueue()
  {
    QueuedGoal candidate;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (validation_in_progress_ || pending_validation_.empty()) return;
      if (!validation_client_->service_is_ready()) return;
      candidate = pending_validation_.front();
      pending_validation_.pop_front();
      validation_in_progress_ = true;
    }

    auto request = std::make_shared<ValidateGoal::Request>();
    request->target = candidate.goal.target;
    validation_client_->async_send_request(
      request,
      [this, candidate](rclcpp::Client<ValidateGoal>::SharedFuture future) {
        const auto response = future.get();
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          validation_in_progress_ = false;
          if (response->valid) approved_goals_.push_back(candidate);
        }
        printLine(
          "goal #" + std::to_string(candidate.id) +
          (response->valid ? " APPROVED: " : " REJECTED: ") + response->reason);
      });
  }

  void dispatchNextGoal()
  {
    QueuedGoal next;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (takeoff_in_progress_ || goal_request_in_progress_ || active_goal_.has_value() ||
        approved_goals_.empty())
      {
        return;
      }
      if (!action_client_->action_server_is_ready()) return;
      next = approved_goals_.front();
      approved_goals_.pop_front();
      active_goal_ = next;
      goal_request_in_progress_ = true;
    }

    rclcpp_action::Client<NavigateTo>::SendGoalOptions options;
    options.goal_response_callback =
      [this, goal_id = next.id](const NavigateGoalHandle::SharedPtr & goal_handle) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          goal_request_in_progress_ = false;
          active_goal_handle_ = goal_handle;
          if (!goal_handle) active_goal_.reset();
        }
        if (goal_handle) {
          printLine("goal #" + std::to_string(goal_id) + " STARTED in FW fast mode");
        } else {
          printLine("goal #" + std::to_string(goal_id) + " REJECTED by action server");
        }
      };
    options.feedback_callback =
      [this, goal_id = next.id](
      NavigateGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateTo::Feedback> feedback)
      {
        if (!feedback->apf_mode.empty()) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          active_apf_mode_ = feedback->apf_mode;
        }
        const auto current_time = std::chrono::steady_clock::now();
        if (last_feedback_print_.time_since_epoch().count() != 0 &&
          std::chrono::duration<double>(current_time - last_feedback_print_).count() <
          feedback_period_s_)
        {
          return;
        }
        last_feedback_print_ = current_time;
        std::ostringstream status;
        status << std::fixed << std::setprecision(1)
               << "  goal #" << goal_id << " | " << feedback->phase
               << " | APF=" << (feedback->avoidance_active ? "ACTIVE" : "clear")
               << " | mode=" << (feedback->apf_mode.empty() ? "unknown" : feedback->apf_mode)
               << '\n'
               << "  position=(" << feedback->position_enu.x << ", "
               << feedback->position_enu.y << ", " << feedback->position_enu.z << ")"
               << " | remaining=" << feedback->remaining_distance_m << "m\n"
               << "  velocity=(" << feedback->velocity_enu.x << ", "
               << feedback->velocity_enu.y << ", " << feedback->velocity_enu.z << ")"
               << " | speed=" << feedback->speed_m_s << "m/s";
        setLiveFeedback(status.str());
      };
    options.result_callback =
      [this, goal_id = next.id](const NavigateGoalHandle::WrappedResult & result) {
        std::string status;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: status = "SUCCEEDED"; break;
          case rclcpp_action::ResultCode::CANCELED: status = "CANCELED"; break;
          case rclcpp_action::ResultCode::ABORTED: status = "ABORTED"; break;
          default: status = "UNKNOWN"; break;
        }
        const std::string message = result.result ? result.result->message : "no result";
        bool another_goal_waiting = false;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          active_goal_.reset();
          active_goal_handle_.reset();
          another_goal_waiting = validation_in_progress_ || !pending_validation_.empty() ||
            !approved_goals_.empty();
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          setLiveFeedback(another_goal_waiting ?
            "  Goal reached; starting the next queued goal." :
            "  Goal reached; transitioning to MC altitude hold.");
        } else {
          setLiveFeedback("  No active navigation goal.");
        }
        printLine(
          "goal #" + std::to_string(goal_id) + " " + status + ": " + message);
      };
    action_client_->async_send_goal(next.goal, options);
  }

  void cancelActiveGoal()
  {
    NavigateGoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      handle = active_goal_handle_;
    }
    if (!handle) {
      printLine("no active goal to cancel");
      return;
    }
    action_client_->async_cancel_goal(handle);
    printLine("cancel requested for active goal");
  }

  void clearWaitingGoals()
  {
    std::size_t removed = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      removed = pending_validation_.size() + approved_goals_.size();
      pending_validation_.clear();
      approved_goals_.clear();
    }
    printLine("cleared " + std::to_string(removed) + " waiting goal(s)");
  }

  void printQueue()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const std::size_t waiting = pending_validation_.size() + approved_goals_.size() +
      static_cast<std::size_t>(validation_in_progress_);
    const std::string active = active_goal_ ? std::to_string(active_goal_->id) : "none";
    printLine(
      "active goal: " + active + " | waiting/validating: " + std::to_string(waiting));
  }

  void printStatus()
  {
    VehicleState state;
    bool available = false;
    std::string apf_mode;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      state = state_;
      available = have_state_;
      apf_mode = active_apf_mode_;
    }
    if (!available) {
      printLine(
        "vehicle state is not available | APF mode=" +
        (apf_mode.empty() ? "unknown" : apf_mode));
      return;
    }
    const double speed = std::sqrt(
      state.velocity_enu.x * state.velocity_enu.x +
      state.velocity_enu.y * state.velocity_enu.y +
      state.velocity_enu.z * state.velocity_enu.z);
    std::ostringstream line;
    line << std::fixed << std::setprecision(1)
         << "vehicle | pos=(" << state.position_enu.x << ", "
         << state.position_enu.y << ", " << state.position_enu.z << ")"
         << " | vel=(" << state.velocity_enu.x << ", "
         << state.velocity_enu.y << ", " << state.velocity_enu.z << ")"
         << " | speed=" << speed << "m/s"
         << " | armed=" << (state.armed ? "yes" : "no")
         << " | offboard=" << (state.offboard ? "yes" : "no")
         << " | APF mode=" << (apf_mode.empty() ? "unknown" : apf_mode);
    printLine(line.str());
  }

  void onTimer()
  {
    processValidationQueue();
    dispatchNextGoal();
  }

  double cruise_speed_m_s_ {20.0};
  double feedback_period_s_ {0.5};
  double takeoff_altitude_m_ {15.0};
  double takeoff_climb_speed_m_s_ {2.0};
  std::atomic<double> takeoff_target_altitude_m_ {15.0};
  std::atomic<bool> running_ {false};
  std::atomic<bool> awaiting_coordinates_ {false};
  std::atomic<bool> interactive_terminal_ {false};
  std::thread input_thread_;
  std::mutex data_mutex_;
  std::mutex output_mutex_;
  VehicleState state_;
  std::string active_apf_mode_;
  bool have_state_ {false};
  bool validation_in_progress_ {false};
  bool goal_request_in_progress_ {false};
  bool takeoff_in_progress_ {false};
  uint64_t next_goal_id_ {1};
  std::deque<QueuedGoal> pending_validation_;
  std::deque<QueuedGoal> approved_goals_;
  std::optional<QueuedGoal> active_goal_;
  NavigateGoalHandle::SharedPtr active_goal_handle_;
  TakeoffGoalHandle::SharedPtr takeoff_goal_handle_;
  std::chrono::steady_clock::time_point last_feedback_print_;
  rclcpp_action::Client<NavigateTo>::SharedPtr action_client_;
  rclcpp_action::Client<Takeoff>::SharedPtr takeoff_client_;
  rclcpp::Client<ValidateGoal>::SharedPtr validation_client_;
  rclcpp::Client<Arm>::SharedPtr arm_client_;
  rclcpp::Client<SetBool>::SharedPtr apf_enabled_client_;
  rclcpp::Client<SetApfMode>::SharedPtr apf_mode_client_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<ApfTelemetry>::SharedPtr apf_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  static constexpr std::size_t max_recent_messages_ {8};
  std::deque<std::string> recent_messages_;
  std::string live_feedback_;
  std::string input_buffer_;
  termios original_terminal_ {};
  bool terminal_mode_active_ {false};
  int escape_sequence_state_ {0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationClientNode>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
