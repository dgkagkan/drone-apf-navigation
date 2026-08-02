#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <drone_interfaces/msg/apf_telemetry.hpp>
#include <drone_interfaces/msg/vehicle_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "drone_navigation/nominal_path_planner.hpp"

using ApfTelemetry = drone_interfaces::msg::ApfTelemetry;
using Marker = visualization_msgs::msg::Marker;
using VehicleState = drone_interfaces::msg::VehicleState;
using drone_navigation::NominalPathParameters;
using drone_navigation::NominalPathPoint;
using drone_navigation::generateNominalPath;
using std::placeholders::_1;

class ApfVisualizerNode : public rclcpp::Node
{
public:
  ApfVisualizerNode()
  : Node("apf_visualizer")
  {
    constexpr double degrees_to_radians = 0.017453292519943295;
    nominal_path_parameters_.max_bank_rad = std::clamp(
      declare_parameter<double>("fw_max_bank_deg", 50.0), 1.0, 75.0) *
      degrees_to_radians;
    nominal_path_parameters_.max_pitch_rad = std::clamp(
      declare_parameter<double>("fw_max_pitch_deg", 13.988346309184921),
      0.0, 45.0) * degrees_to_radians;
    nominal_path_parameters_.sample_distance_m = std::max(
      0.2, declare_parameter<double>("nominal_path_sample_distance_m", 2.0));
    nominal_path_parameters_.arrival_radius_m = std::max(
      0.2, declare_parameter<double>("goal_tolerance_m", 25.0));
    nominal_path_parameters_.altitude_tolerance_m = std::max(
      0.2, declare_parameter<double>("altitude_tolerance_m", 2.0));
    nominal_path_parameters_.max_points = static_cast<std::size_t>(std::clamp<int64_t>(
      declare_parameter<int64_t>("nominal_path_max_points", 3000), 100, 10000));
    nominal_path_min_fw_speed_m_s_ = std::max(
      0.5, declare_parameter<double>("nominal_path_min_fw_speed_m_s", 5.0));
    force_scale_ = std::max(0.1, declare_parameter<double>("force_scale", 6.0));
    max_arrow_length_m_ = std::max(
      0.1, declare_parameter<double>("max_arrow_length", 3.0));
    smoothing_alpha_ = std::clamp(
      declare_parameter<double>("force_smoothing_alpha", 0.2), 0.01, 1.0);
    path_sample_distance_m_ = std::max(
      0.1, declare_parameter<double>("path_sample_distance", 0.5));

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_sub_ = create_subscription<VehicleState>(
      "/vehicle/state", 10, std::bind(&ApfVisualizerNode::onState, this, _1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/navigation/active_goal", path_qos,
      std::bind(&ApfVisualizerNode::onGoal, this, _1));
    telemetry_sub_ = create_subscription<ApfTelemetry>(
      "/apf/telemetry", rclcpp::QoS(1).best_effort(),
      std::bind(&ApfVisualizerNode::onTelemetry, this, _1));
    force_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/apf/forces", rclcpp::QoS(1).best_effort());
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/navigation/flown_path", path_qos);
    nominal_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/navigation/nominal_path", path_qos);
  }

private:
  static std_msgs::msg::ColorRGBA color(float red, float green, float blue)
  {
    std_msgs::msg::ColorRGBA value;
    value.r = red;
    value.g = green;
    value.b = blue;
    value.a = 1.0F;
    return value;
  }

  static geometry_msgs::msg::Point point(double x, double y, double z)
  {
    geometry_msgs::msg::Point value;
    value.x = x;
    value.y = y;
    value.z = z;
    return value;
  }

  static geometry_msgs::msg::PoseStamped pose(
    const std_msgs::msg::Header & header, const geometry_msgs::msg::Point & position)
  {
    geometry_msgs::msg::PoseStamped value;
    value.header = header;
    value.pose.position = position;
    value.pose.orientation.w = 1.0;
    return value;
  }

  geometry_msgs::msg::Vector3 displayedVector(
    const geometry_msgs::msg::Vector3 & input) const
  {
    geometry_msgs::msg::Vector3 output;
    output.x = input.x * force_scale_;
    output.y = input.y * force_scale_;
    output.z = input.z * force_scale_;
    const double length = std::sqrt(
      output.x * output.x + output.y * output.y + output.z * output.z);
    if (length > max_arrow_length_m_) {
      const double scale = max_arrow_length_m_ / length;
      output.x *= scale;
      output.y *= scale;
      output.z *= scale;
    }
    return output;
  }

  Marker arrow(
    int id, const std::string & name, const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Point & origin,
    const geometry_msgs::msg::Vector3 & vector,
    const std_msgs::msg::ColorRGBA & arrow_color) const
  {
    Marker marker;
    marker.header = header;
    marker.ns = name;
    marker.id = id;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.24;
    marker.scale.y = 0.54;
    marker.scale.z = 0.66;
    marker.color = arrow_color;
    const auto displayed = displayedVector(vector);
    const double length = std::sqrt(
      displayed.x * displayed.x + displayed.y * displayed.y + displayed.z * displayed.z);
    if (length < 0.05) {
      marker.action = Marker::DELETE;
      return marker;
    }
    marker.points.push_back(origin);
    marker.points.push_back(point(
      origin.x + displayed.x, origin.y + displayed.y, origin.z + displayed.z));
    return marker;
  }

  void initializeActualPath(const VehicleState & state)
  {
    const auto start = state.position_enu;
    actual_path_.header = state.header;
    actual_path_.poses.push_back(pose(state.header, start));
    last_path_point_ = start;
    actual_path_initialized_ = true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    active_goal_ = *goal;
    have_goal_ = true;
    nominal_path_needs_update_ = true;
    clearNominalPath();
    if (nominalPathStartReady()) publishNominalPath();
  }

  void clearNominalPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    nominal_path_pub_->publish(path);
  }

  double horizontalSpeed() const
  {
    return std::hypot(
      latest_state_.velocity_enu.x, latest_state_.velocity_enu.y);
  }

  bool nominalPathStartReady() const
  {
    if (!have_state_ || !latest_state_.position_valid ||
      latest_state_.vehicle_mode != VehicleState::MODE_FIXED_WING)
    {
      return false;
    }
    const double speed_m_s = horizontalSpeed();
    return std::isfinite(speed_m_s) && speed_m_s >= nominal_path_min_fw_speed_m_s_;
  }

  void publishNominalPath()
  {
    const auto & start_position = latest_state_.position_enu;
    const auto & goal_position = active_goal_.pose.position;
    const double speed_m_s = horizontalSpeed();
    const double course_enu_rad = speed_m_s >= nominal_path_min_fw_speed_m_s_ ?
      std::atan2(latest_state_.velocity_enu.y, latest_state_.velocity_enu.x) :
      1.5707963267948966 - latest_state_.heading_ned_rad;
    auto path_parameters = nominal_path_parameters_;
    path_parameters.speed_m_s = std::max(0.1, speed_m_s);
    const auto points = generateNominalPath(
      NominalPathPoint{
        start_position.x, start_position.y, start_position.z, course_enu_rad},
      NominalPathPoint{
        goal_position.x, goal_position.y, goal_position.z, course_enu_rad},
      path_parameters);

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    path.poses.reserve(points.size());
    for (const auto & path_point : points) {
      path.poses.push_back(pose(
        path.header, point(path_point.x, path_point.y, path_point.z)));
    }
    nominal_path_pub_->publish(path);
    nominal_path_needs_update_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Nominal FW path starts at %.1fm/s with course %.1fdeg and max bank %.1fdeg",
      speed_m_s, course_enu_rad * 57.29577951308232,
      path_parameters.max_bank_rad * 57.29577951308232);

    const auto & last = points.back();
    const double remaining_m = std::hypot(
      goal_position.x - last.x, goal_position.y - last.y);
    if (remaining_m > nominal_path_parameters_.arrival_radius_m) {
      RCLCPP_WARN(
        get_logger(),
        "Nominal path reached its %zu-point limit with %.1fm remaining",
        points.size(), remaining_m);
    }
  }

  void onState(const VehicleState::SharedPtr state)
  {
    if (!state->position_valid) return;
    latest_state_ = *state;
    have_state_ = true;
    if (!actual_path_initialized_) initializeActualPath(*state);
    if (have_goal_ && nominal_path_needs_update_ && nominalPathStartReady()) {
      publishNominalPath();
    }
    const auto & current = state->position_enu;
    const double distance = std::sqrt(
      std::pow(current.x - last_path_point_.x, 2.0) +
      std::pow(current.y - last_path_point_.y, 2.0) +
      std::pow(current.z - last_path_point_.z, 2.0));
    if (distance >= path_sample_distance_m_) {
      actual_path_.poses.push_back(pose(state->header, current));
      actual_path_.header = state->header;
      last_path_point_ = current;
    }
    if ((now() - last_path_publish_).seconds() >= 0.1) {
      last_path_publish_ = now();
      actual_path_pub_->publish(actual_path_);
    }
  }

  void smooth(
    geometry_msgs::msg::Vector3 & previous,
    const geometry_msgs::msg::Vector3 & current) const
  {
    previous.x += smoothing_alpha_ * (current.x - previous.x);
    previous.y += smoothing_alpha_ * (current.y - previous.y);
    previous.z += smoothing_alpha_ * (current.z - previous.z);
  }

  void onTelemetry(const ApfTelemetry::SharedPtr telemetry)
  {
    if (!have_state_) return;
    if (!forces_initialized_) {
      attractive_ = telemetry->attractive_force_enu;
      repulsive_ = telemetry->repulsive_force_enu;
      safe_ = telemetry->safe_command_enu;
      forces_initialized_ = true;
    } else {
      smooth(attractive_, telemetry->attractive_force_enu);
      smooth(repulsive_, telemetry->repulsive_force_enu);
      smooth(safe_, telemetry->safe_command_enu);
    }

    const auto origin = latest_state_.position_enu;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(arrow(
      0, "attractive", telemetry->header, origin, attractive_, color(0.1F, 1.0F, 0.2F)));
    markers.markers.push_back(arrow(
      1, "repulsive", telemetry->header, origin, repulsive_, color(1.0F, 0.15F, 0.1F)));
    markers.markers.push_back(arrow(
      2, "resultant", telemetry->header, origin, safe_, color(1.0F, 0.85F, 0.1F)));
    force_pub_->publish(markers);
  }

  NominalPathParameters nominal_path_parameters_;
  double force_scale_ {6.0};
  double max_arrow_length_m_ {3.0};
  double smoothing_alpha_ {0.2};
  double path_sample_distance_m_ {0.5};
  double nominal_path_min_fw_speed_m_s_ {5.0};
  bool have_state_ {false};
  bool actual_path_initialized_ {false};
  bool have_goal_ {false};
  bool nominal_path_needs_update_ {false};
  bool forces_initialized_ {false};
  VehicleState latest_state_;
  geometry_msgs::msg::PoseStamped active_goal_;
  geometry_msgs::msg::Vector3 attractive_;
  geometry_msgs::msg::Vector3 repulsive_;
  geometry_msgs::msg::Vector3 safe_;
  geometry_msgs::msg::Point last_path_point_;
  rclcpp::Time last_path_publish_ {0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path actual_path_;
  rclcpp::Subscription<VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<ApfTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr force_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nominal_path_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApfVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
