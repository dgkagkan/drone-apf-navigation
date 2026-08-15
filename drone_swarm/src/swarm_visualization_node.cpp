#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <drone_interfaces/msg/swarm_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Path = nav_msgs::msg::Path;
using SwarmState = drone_interfaces::msg::SwarmState;

namespace drone_swarm
{

class SwarmVisualizationNode : public rclcpp::Node
{
public:
  SwarmVisualizationNode()
  : Node("swarm_visualization")
  {
    force_pub_ = create_publisher<MarkerArray>(
      "/swarm/apf/forces", rclcpp::QoS(10).best_effort());
    const auto path_qos = rclcpp::QoS(10).reliable().transient_local();
    nominal_path_pub_ = create_publisher<MarkerArray>(
      "/swarm/navigation/nominal_paths", path_qos);
    flown_path_pub_ = create_publisher<MarkerArray>(
      "/swarm/navigation/flown_paths", path_qos);
    state_sub_ = create_subscription<SwarmState>(
      "/swarm/state", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&SwarmVisualizationNode::onState, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Dynamic swarm APF and path visualization is ready");
  }

private:
  struct DroneVisualization
  {
    std::string drone_namespace;
    bool connected {false};
    rclcpp::Subscription<MarkerArray>::SharedPtr force_sub;
    rclcpp::Subscription<Path>::SharedPtr nominal_path_sub;
    rclcpp::Subscription<Path>::SharedPtr flown_path_sub;
  };

  static std::array<float, 3> droneColor(const std::string & drone_id)
  {
    static constexpr std::array<std::array<float, 3>, 6> colors {{
      {{1.0F, 0.20F, 0.20F}},
      {{0.20F, 1.0F, 0.20F}},
      {{0.20F, 0.50F, 1.0F}},
      {{1.0F, 0.65F, 0.15F}},
      {{0.75F, 0.30F, 1.0F}},
      {{0.10F, 0.90F, 0.90F}},
    }};
    uint32_t hash = 2166136261U;
    for (const unsigned char character : drone_id) {
      hash = (hash ^ character) * 16777619U;
    }
    return colors[hash % colors.size()];
  }

  void onState(const SwarmState::SharedPtr state)
  {
    for (const auto & drone : state->drones) {
      if (!drone.registered || drone.drone_namespace.empty()) continue;
      auto found = drones_.find(drone.drone_id);
      if (found == drones_.end()) {
        found = drones_.emplace(
          drone.drone_id, createDroneVisualization(
            drone.drone_id, drone.drone_namespace)).first;
      }
      auto & visualization = found->second;
      if (visualization.connected && !drone.connected) clearDrone(drone.drone_id);
      visualization.connected = drone.connected;
    }
  }

  DroneVisualization createDroneVisualization(
    const std::string & drone_id, const std::string & drone_namespace)
  {
    DroneVisualization visualization;
    visualization.drone_namespace = drone_namespace;
    visualization.force_sub = create_subscription<MarkerArray>(
      drone_namespace + "/apf/forces", rclcpp::QoS(1).best_effort(),
      [this, drone_id](const MarkerArray::SharedPtr markers) {
        if (!droneConnected(drone_id)) return;
        MarkerArray output = *markers;
        for (auto & marker : output.markers) {
          marker.ns = drone_id + "/" + marker.ns;
        }
        force_pub_->publish(output);
      });
    const auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    visualization.nominal_path_sub = create_subscription<Path>(
      drone_namespace + "/navigation/nominal_path", path_qos,
      [this, drone_id](const Path::SharedPtr path) {
        if (droneConnected(drone_id)) publishPath(drone_id, *path, true);
      });
    visualization.flown_path_sub = create_subscription<Path>(
      drone_namespace + "/navigation/flown_path", path_qos,
      [this, drone_id](const Path::SharedPtr path) {
        if (droneConnected(drone_id)) publishPath(drone_id, *path, false);
      });
    RCLCPP_INFO(
      get_logger(), "Visualizing %s from namespace %s",
      drone_id.c_str(), drone_namespace.c_str());
    return visualization;
  }

  bool droneConnected(const std::string & drone_id) const
  {
    const auto found = drones_.find(drone_id);
    return found != drones_.end() && found->second.connected;
  }

  void publishPath(const std::string & drone_id, const Path & path, bool nominal)
  {
    Marker marker;
    marker.header = path.header;
    marker.ns = drone_id + (nominal ? "/nominal_path" : "/flown_path");
    marker.id = 0;
    marker.type = Marker::LINE_STRIP;
    marker.action = path.poses.empty() ? Marker::DELETE : Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = nominal ? 0.12 : 0.18;
    const auto color = droneColor(drone_id);
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = nominal ? 0.65F : 1.0F;
    marker.points.reserve(path.poses.size());
    for (const auto & pose : path.poses) marker.points.push_back(pose.pose.position);
    MarkerArray output;
    output.markers.push_back(std::move(marker));
    (nominal ? nominal_path_pub_ : flown_path_pub_)->publish(output);
  }

  void clearDrone(const std::string & drone_id)
  {
    MarkerArray forces;
    const std::array<std::string, 3> names {"attractive", "repulsive", "resultant"};
    for (std::size_t index = 0; index < names.size(); ++index) {
      Marker marker;
      marker.ns = drone_id + "/" + names[index];
      marker.id = static_cast<int>(index);
      marker.action = Marker::DELETE;
      forces.markers.push_back(std::move(marker));
    }
    force_pub_->publish(forces);

    Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = "map";
    publishPath(drone_id, empty_path, true);
    publishPath(drone_id, empty_path, false);
  }

  std::map<std::string, DroneVisualization> drones_;
  rclcpp::Subscription<SwarmState>::SharedPtr state_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr force_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr nominal_path_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr flown_path_pub_;
};

}  // namespace drone_swarm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_swarm::SwarmVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
