#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <drone_interfaces/msg/swarm_map_cloud.hpp>
#include <drone_interfaces/msg/swarm_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "drone_swarm/global_map_fusion.hpp"

namespace drone_swarm
{

using SwarmMapCloud = drone_interfaces::msg::SwarmMapCloud;
using SwarmState = drone_interfaces::msg::SwarmState;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

class SwarmGlobalMapServer : public rclcpp::Node
{
public:
  SwarmGlobalMapServer()
  : Node("swarm_global_map_server")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    mapping_topic_prefix_ = declare_parameter<std::string>(
      "mapping_topic_prefix", "/swarm");
    resolution_ = std::max(
      0.05, declare_parameter<double>("resolution", 0.5));
    hit_probability_ = std::clamp(
      declare_parameter<double>("hit_probability", 0.75), 0.001, 0.999);
    miss_probability_ = std::clamp(
      declare_parameter<double>("miss_probability", 0.45), 0.001, 0.999);
    min_probability_ = std::clamp(
      declare_parameter<double>("min_probability", 0.12), 0.001, 0.499);
    max_probability_ = std::clamp(
      declare_parameter<double>("max_probability", 0.97), 0.501, 0.999);
    occupied_probability_ = std::clamp(
      declare_parameter<double>("occupied_probability", 0.5), 0.001, 0.999);
    mapping_queue_size_ = std::clamp(
      static_cast<int>(declare_parameter<int>("mapping_queue_size", 5)), 1, 20);
    publish_rate_hz_ = std::clamp(
      declare_parameter<double>("publish_rate_hz", 2.0), 0.1, 10.0);
    octomap_publish_rate_hz_ = std::clamp(
      declare_parameter<double>("octomap_publish_rate_hz", 1.0), 0.1, 10.0);
    max_ray_length_m_ = std::max(
      resolution_, declare_parameter<double>("max_ray_length_m", 300.0));
    max_cloud_age_sec_ = std::max(
      0.0, declare_parameter<double>("max_cloud_age_sec", 0.0));
    remove_submap_on_disconnect_ = declare_parameter<bool>(
      "remove_submap_on_disconnect", false);
    retain_submap_on_disconnect_ = declare_parameter<bool>(
      "retain_submap_on_disconnect", true);
    submap_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("submap_timeout_sec", 30.0));
    octree_ = createOctree();

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    state_sub_ = create_subscription<SwarmState>(
      "/swarm/state", state_qos,
      std::bind(&SwarmGlobalMapServer::onSwarmState, this, std::placeholders::_1));

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    global_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm/octomap_point_cloud_centers", map_qos);
    global_binary_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/swarm/octomap_binary", map_qos);
    global_full_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/swarm/octomap_full", map_qos);
    mapping_debug_pub_ = create_publisher<MarkerArray>(
      "/swarm/mapping_debug", map_qos);

    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&SwarmGlobalMapServer::publishMaps, this));
    timeout_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SwarmGlobalMapServer::expireRetainedSubmaps, this));

    RCLCPP_INFO(
      get_logger(),
      "Dynamic global map ready: frame=%s resolution=%.2fm queue=%d publish=%.1fHz "
      "dynamic workers octomap serialization=%.1fHz max ray=%.1fm "
      "subscriptions follow /swarm/state",
      map_frame_.c_str(), resolution_, mapping_queue_size_, publish_rate_hz_,
      octomap_publish_rate_hz_, max_ray_length_m_);

    octomap_publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / octomap_publish_rate_hz_),
      std::bind(&SwarmGlobalMapServer::requestOctomapPublish, this));
    octomap_publish_thread_ = std::thread(
      &SwarmGlobalMapServer::octomapPublishLoop, this);
  }

  ~SwarmGlobalMapServer() override
  {
    {
      std::lock_guard<std::mutex> lock(mapping_queue_mutex_);
      mapping_worker_shutdown_ = true;
    }
    mapping_queue_condition_.notify_all();
    for (auto & worker : mapping_worker_threads_) {
      if (worker.joinable()) worker.join();
    }

    {
      std::lock_guard<std::mutex> lock(octomap_publish_mutex_);
      octomap_publish_shutdown_ = true;
    }
    octomap_publish_condition_.notify_one();
    if (octomap_publish_thread_.joinable()) octomap_publish_thread_.join();
  }

private:
  struct DroneMapping
  {
    std::string boot_id;
    bool registered {false};
    bool connected {false};
    bool has_lidar {false};
    std::chrono::steady_clock::time_point disconnected_at;
    rclcpp::Subscription<SwarmMapCloud>::SharedPtr cloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub;
    std::uint64_t received_clouds {0};
    std::uint64_t processed_clouds {0};
    std::uint64_t dropped_invalid {0};
    std::uint64_t dropped_stale {0};
    std::int64_t last_stamp_ns {0};
    bool worker_started {false};
    mutable std::mutex local_map_mutex;
    std::unique_ptr<octomap::OcTree> local_octree;
  };

  std::unique_ptr<octomap::OcTree> createOctree() const
  {
    auto tree = std::make_unique<octomap::OcTree>(resolution_);
    tree->setProbHit(hit_probability_);
    tree->setProbMiss(miss_probability_);
    tree->setClampingThresMin(min_probability_);
    tree->setClampingThresMax(max_probability_);
    tree->setOccupancyThres(occupied_probability_);
    return tree;
  }

  static const sensor_msgs::msg::PointField * findField(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    const auto found = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [&name](const auto & field) {return field.name == name;});
    return found == cloud.fields.end() ? nullptr : &(*found);
  }

  static bool readCoordinate(
    const std::uint8_t * point,
    const std::size_t point_step,
    const sensor_msgs::msg::PointField & field,
    double & value)
  {
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      if (field.offset + sizeof(float) > point_step) return false;
      float coordinate = 0.0F;
      std::memcpy(&coordinate, point + field.offset, sizeof(coordinate));
      value = coordinate;
      return std::isfinite(value);
    }
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
      if (field.offset + sizeof(double) > point_step) return false;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return std::isfinite(value);
    }
    return false;
  }

  static octomap::OcTreeKey toOctomapKey(const VoxelKey & key)
  {
    return octomap::OcTreeKey(
      static_cast<octomap::key_type>(key.x),
      static_cast<octomap::key_type>(key.y),
      static_cast<octomap::key_type>(key.z));
  }

  std::string cloudTopic(const std::string & drone_id) const
  {
    return mapping_topic_prefix_ + "/" + drone_id + "/map_cloud";
  }

  std::string debugTopic(const std::string & drone_id) const
  {
    return mapping_topic_prefix_ + "/mapping/" + drone_id + "/occupied_voxels";
  }

  void onSwarmState(const SwarmState::SharedPtr state)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    const auto now = std::chrono::steady_clock::now();

    for (const auto & drone : state->drones) {
      if (drone.drone_id.empty()) continue;
      auto & mapping = drones_[drone.drone_id];

      if (!mapping.boot_id.empty() && !drone.boot_id.empty() &&
        mapping.boot_id != drone.boot_id)
      {
        std::lock_guard<std::mutex> local_map_lock(mapping.local_map_mutex);
        if (mapping.local_octree) mapping.local_octree->clear();
        mapping.last_stamp_ns = 0;
        RCLCPP_INFO(
          get_logger(), "Reset mapping contribution for %s after boot/session change",
          drone.drone_id.c_str());
      }
      if (!drone.boot_id.empty()) mapping.boot_id = drone.boot_id;
      const bool should_subscribe =
        drone.registered && drone.connected && drone.has_lidar;
      const bool was_connected = mapping.connected;
      mapping.registered = drone.registered;
      mapping.connected = should_subscribe;
      mapping.has_lidar = drone.has_lidar;

      if (should_subscribe) {
        mapping.disconnected_at = std::chrono::steady_clock::time_point{};
        if (!mapping.cloud_sub) createDroneSubscription(drone.drone_id, mapping);
      } else {
        if (was_connected || mapping.cloud_sub) {
          mapping.cloud_sub.reset();
          mapping.disconnected_at = now;
        }
        if (!retain_submap_on_disconnect_ || remove_submap_on_disconnect_) {
          std::lock_guard<std::mutex> local_map_lock(mapping.local_map_mutex);
          if (mapping.local_octree) mapping.local_octree->clear();
        }
      }
    }
  }

  void createDroneSubscription(const std::string & drone_id, DroneMapping & mapping)
  {
    {
      std::lock_guard<std::mutex> local_map_lock(mapping.local_map_mutex);
      if (!mapping.local_octree) mapping.local_octree = createOctree();
    }
    const auto qos = rclcpp::SensorDataQoS().keep_last(mapping_queue_size_);
    mapping.cloud_sub = create_subscription<SwarmMapCloud>(
      cloudTopic(drone_id), qos,
      [this, drone_id](const SwarmMapCloud::SharedPtr message) {
        onMapCloud(drone_id, message);
    });
    mapping.debug_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      debugTopic(drone_id), rclcpp::QoS(1).reliable().transient_local());
    if (!mapping.worker_started) {
      mapping.worker_started = true;
      mapping_worker_threads_.emplace_back(
        &SwarmGlobalMapServer::mappingWorkerLoop, this, drone_id, &mapping);
      RCLCPP_INFO(
        get_logger(), "Dynamic mapping worker created for %s (workers=%zu)",
        drone_id.c_str(), mapping_worker_threads_.size());
    }
    RCLCPP_INFO(
      get_logger(), "Mapping subscription added dynamically: %s -> %s",
      drone_id.c_str(), cloudTopic(drone_id).c_str());
  }

  void onMapCloud(const std::string & expected_drone_id, const SwarmMapCloud::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      const auto found = drones_.find(expected_drone_id);
      if (found == drones_.end() || !found->second.connected) return;
      ++found->second.received_clouds;
    }

    {
      std::lock_guard<std::mutex> lock(mapping_queue_mutex_);
      // Keep the newest scan for each drone. This prevents a slow map update
      // from building a stale queue while the live visualization keeps running.
      pending_map_clouds_[expected_drone_id] = message;
    }
    mapping_queue_condition_.notify_all();
  }

  void mappingWorkerLoop(
    const std::string & worker_drone_id, DroneMapping * mapping)
  {
    // Each mapping-capable drone owns one worker. This keeps clouds from
    // different drones independent while preventing concurrent updates from
    // the same drone from being applied out of order.
    while (true) {
      SwarmMapCloud::SharedPtr message;
      {
        std::unique_lock<std::mutex> lock(mapping_queue_mutex_);
        mapping_queue_condition_.wait(lock, [this, &worker_drone_id]() {
          return mapping_worker_shutdown_ ||
                 pending_map_clouds_.find(worker_drone_id) != pending_map_clouds_.end();
        });
        if (mapping_worker_shutdown_) return;

        const auto candidate = pending_map_clouds_.find(worker_drone_id);
        if (candidate == pending_map_clouds_.end()) continue;
        message = std::move(candidate->second);
        pending_map_clouds_.erase(candidate);
      }

      try {
        processMapCloud(worker_drone_id, *mapping, message);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          get_logger(), "Mapping worker failed for %s: %s",
          worker_drone_id.c_str(), exception.what());
      }
    }
  }

  void recordDroppedCloud(const std::string & drone_id, const bool stale)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    const auto found = drones_.find(drone_id);
    if (found == drones_.end()) return;
    if (stale) {
      ++found->second.dropped_stale;
    } else {
      ++found->second.dropped_invalid;
    }
  }

  void processMapCloud(
    const std::string & expected_drone_id, DroneMapping & mapping,
    const SwarmMapCloud::SharedPtr message)
  {
    if (!message) return;

    if (message->drone_id != expected_drone_id ||
      message->header.frame_id != map_frame_ ||
      message->cloud.header.frame_id != map_frame_ ||
      message->cloud.is_bigendian || message->cloud.point_step == 0)
    {
      recordDroppedCloud(expected_drone_id, false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping invalid map cloud for %s: expected id/frame (%s/%s), got (%s/%s)",
        expected_drone_id.c_str(), expected_drone_id.c_str(), map_frame_.c_str(),
        message->drone_id.c_str(), message->cloud.header.frame_id.c_str());
      return;
    }

    const auto stamp = rclcpp::Time(message->cloud.header.stamp);
    const auto current_time = get_clock()->now();
    if (max_cloud_age_sec_ > 0.0 && stamp.nanoseconds() > 0 &&
      current_time.nanoseconds() > stamp.nanoseconds() &&
      (current_time - stamp).seconds() > max_cloud_age_sec_)
    {
      recordDroppedCloud(expected_drone_id, true);
      return;
    }

    const auto * x_field = findField(message->cloud, "x");
    const auto * y_field = findField(message->cloud, "y");
    const auto * z_field = findField(message->cloud, "z");
    if (!x_field || !y_field || !z_field) {
      recordDroppedCloud(expected_drone_id, false);
      return;
    }

    const double origin_x = message->sensor_origin.x;
    const double origin_y = message->sensor_origin.y;
    const double origin_z = message->sensor_origin.z;
    if (!std::isfinite(origin_x) || !std::isfinite(origin_y) ||
      !std::isfinite(origin_z))
    {
      recordDroppedCloud(expected_drone_id, false);
      return;
    }

    double reported_max_range = message->max_range_m;
    if (!std::isfinite(reported_max_range) || reported_max_range <= 0.0) {
      reported_max_range = max_ray_length_m_;
    }
    const double effective_max_range = std::min(reported_max_range, max_ray_length_m_);
    const octomap::point3d origin(
      static_cast<float>(origin_x), static_cast<float>(origin_y),
      static_cast<float>(origin_z));
    octomap::Pointcloud scan;
    scan.reserve(message->cloud.width * message->cloud.height);
    for (std::size_t row = 0; row < message->cloud.height; ++row) {
      for (std::size_t column = 0; column < message->cloud.width; ++column) {
        const std::size_t offset =
          row * message->cloud.row_step + column * message->cloud.point_step;
        if (offset + message->cloud.point_step > message->cloud.data.size()) continue;
        const auto * point = message->cloud.data.data() + offset;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!readCoordinate(point, message->cloud.point_step, *x_field, x) ||
          !readCoordinate(point, message->cloud.point_step, *y_field, y) ||
          !readCoordinate(point, message->cloud.point_step, *z_field, z))
        {
          continue;
        }

        const double dx = x - origin_x;
        const double dy = y - origin_y;
        const double dz = z - origin_z;
        const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!std::isfinite(range) || range < resolution_ * 0.25 || range < 1.0e-6) {
          continue;
        }

        scan.push_back(
          static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
      }
    }

    if (scan.size() == 0U) return;

    // Use the same optimized batch insertion as the working octomap_server
    // pipeline. The local tree is owned by this drone's worker, so different
    // drones can insert their complete clouds in parallel without sharing an
    // OctoMap instance.
    {
      std::lock_guard<std::mutex> local_map_lock(mapping.local_map_mutex);
      if (!mapping.local_octree) mapping.local_octree = createOctree();
      mapping.local_octree->insertPointCloud(
        scan, origin, effective_max_range, true, false);
      mapping.local_octree->updateInnerOccupancy();
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    const auto found = drones_.find(expected_drone_id);
    if (found == drones_.end() || !found->second.connected) return;
    ++found->second.processed_clouds;
    found->second.last_stamp_ns = stamp.nanoseconds();
    ++updates_since_publish_;
  }

  sensor_msgs::msg::PointCloud2 makeCloud(const std::vector<VoxelKey> & keys) const
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = map_frame_;
    message.header.stamp = get_clock()->now();
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(keys.size());
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    for (const auto & key : keys) {
      const auto point = octree_->keyToCoord(toOctomapKey(key));
      *x = point.x();
      *y = point.y();
      *z = point.z();
      ++x;
      ++y;
      ++z;
    }
    message.is_dense = true;
    return message;
  }

  std::vector<VoxelKey> occupiedKeys(const DroneMapping & mapping) const
  {
    std::vector<VoxelKey> keys;
    std::lock_guard<std::mutex> local_map_lock(mapping.local_map_mutex);
    if (!mapping.local_octree) return keys;

    keys.reserve(mapping.local_octree->size());
    for (auto iterator = mapping.local_octree->begin_leafs(),
      end = mapping.local_octree->end_leafs(); iterator != end; ++iterator)
    {
      if (!mapping.local_octree->isNodeOccupied(*iterator)) continue;
      const auto & key = iterator.getKey();
      keys.push_back({
        static_cast<std::int64_t>(key[0]),
        static_cast<std::int64_t>(key[1]),
        static_cast<std::int64_t>(key[2])});
    }
    return keys;
  }

  void rebuildGlobalTree(const std::vector<VoxelKey> & global_keys)
  {
    octree_->clear();
    for (const auto & key : global_keys) {
      octree_->updateNode(toOctomapKey(key), true, true);
    }
    if (!global_keys.empty()) octree_->updateInnerOccupancy();
  }

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
    std::uint32_t hash = 2166136261U;
    for (const unsigned char character : drone_id) {
      hash = (hash ^ character) * 16777619U;
    }
    return colors[hash % colors.size()];
  }

  MarkerArray makeDebugMarkers(
    const std::map<std::string, std::vector<VoxelKey>> & drone_keys) const
  {
    MarkerArray output;
    Marker clear;
    clear.action = Marker::DELETEALL;
    output.markers.push_back(clear);
    int marker_id = 0;
    for (const auto & [drone_id, mapping] : drones_) {
      const auto found = drone_keys.find(drone_id);
      if (found == drone_keys.end() || found->second.empty()) continue;
      Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = get_clock()->now();
      marker.ns = "mapping/" + drone_id;
      marker.id = marker_id++;
      marker.type = Marker::POINTS;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = static_cast<float>(resolution_ * 0.7);
      marker.scale.y = static_cast<float>(resolution_ * 0.7);
      const auto color = droneColor(drone_id);
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = mapping.connected ? 0.75F : 0.35F;
      marker.points.reserve(found->second.size());
      for (const auto & key : found->second) {
        const auto point = octree_->keyToCoord(toOctomapKey(key));
        geometry_msgs::msg::Point output_point;
        output_point.x = point.x();
        output_point.y = point.y();
        output_point.z = point.z();
        marker.points.push_back(output_point);
      }
      output.markers.push_back(std::move(marker));
    }
    return output;
  }

  void publishMaps()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::map<std::string, std::vector<VoxelKey>> drone_keys;
    std::unordered_set<VoxelKey, VoxelKeyHash> global_key_set;
    for (const auto & [drone_id, mapping] : drones_) {
      auto keys = occupiedKeys(mapping);
      for (const auto & key : keys) global_key_set.insert(key);
      drone_keys.emplace(drone_id, std::move(keys));
    }

    std::vector<VoxelKey> global_keys;
    global_keys.reserve(global_key_set.size());
    for (const auto & key : global_key_set) global_keys.push_back(key);
    rebuildGlobalTree(global_keys);
    global_cloud_pub_->publish(makeCloud(global_keys));

    for (auto & [drone_id, mapping] : drones_) {
      if (!mapping.debug_pub) continue;
      mapping.debug_pub->publish(makeCloud(drone_keys.at(drone_id)));
    }
    mapping_debug_pub_->publish(makeDebugMarkers(drone_keys));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Global map: %zu occupied voxels from %zu active mapping drones, "
      "processed clouds=%zu",
      global_keys.size(), activeDroneCount(),
      updates_since_publish_);
    for (const auto & [drone_id, mapping] : drones_) {
      if (mapping.received_clouds == 0U && mapping.processed_clouds == 0U) continue;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Mapping %s: received=%llu processed=%llu invalid=%llu stale=%llu "
        "last_stamp_ns=%lld contribution_voxels=%zu",
        drone_id.c_str(),
        static_cast<unsigned long long>(mapping.received_clouds),
        static_cast<unsigned long long>(mapping.processed_clouds),
        static_cast<unsigned long long>(mapping.dropped_invalid),
        static_cast<unsigned long long>(mapping.dropped_stale),
        static_cast<long long>(mapping.last_stamp_ns),
        drone_keys.at(drone_id).size());
    }
    updates_since_publish_ = 0;
  }

  void requestOctomapPublish()
  {
    {
      std::lock_guard<std::mutex> lock(octomap_publish_mutex_);
      if (octomap_publish_shutdown_) return;
      octomap_publish_requested_ = true;
    }
    octomap_publish_condition_.notify_one();
  }

  void octomapPublishLoop()
  {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(octomap_publish_mutex_);
        octomap_publish_condition_.wait(lock, [this]() {
          return octomap_publish_requested_ || octomap_publish_shutdown_;
        });
        if (octomap_publish_shutdown_) return;
        octomap_publish_requested_ = false;
      }

      std::unique_ptr<octomap::OcTree> snapshot;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        snapshot = std::make_unique<octomap::OcTree>(*octree_);
      }
      publishOctomapMessages(*snapshot);
    }
  }

  void publishOctomapMessages(const octomap::OcTree & snapshot)
  {
    const auto stamp = get_clock()->now();

    octomap_msgs::msg::Octomap binary;
    binary.header.frame_id = map_frame_;
    binary.header.stamp = stamp;
    if (octomap_msgs::binaryMapToMsg(snapshot, binary)) {
      global_binary_pub_->publish(binary);
    }

    octomap_msgs::msg::Octomap full;
    full.header = binary.header;
    if (octomap_msgs::fullMapToMsg(snapshot, full)) {
      global_full_pub_->publish(full);
    }
  }

  std::size_t activeDroneCount() const
  {
    std::size_t count = 0;
    for (const auto & [unused_id, mapping] : drones_) {
      (void)unused_id;
      if (mapping.registered && mapping.connected && mapping.has_lidar) ++count;
    }
    return count;
  }

  void expireRetainedSubmaps()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!retain_submap_on_disconnect_ || remove_submap_on_disconnect_ ||
      submap_timeout_sec_ <= 0.0)
    {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    for (auto & [drone_id, mapping] : drones_) {
      if (mapping.connected || mapping.disconnected_at.time_since_epoch().count() == 0) {
        continue;
      }
      const double elapsed = std::chrono::duration<double>(
        now - mapping.disconnected_at).count();
      if (elapsed < submap_timeout_sec_) continue;
      mapping.cloud_sub.reset();
      {
        std::lock_guard<std::mutex> local_map_lock(mapping.local_map_mutex);
        if (mapping.local_octree) mapping.local_octree->clear();
      }
      mapping.disconnected_at = std::chrono::steady_clock::time_point{};
      RCLCPP_INFO(
        get_logger(), "Removed retained mapping contribution for %s after %.1fs timeout",
        drone_id.c_str(), elapsed);
    }
  }

  std::string map_frame_;
  std::string mapping_topic_prefix_;
  double resolution_ {0.5};
  double hit_probability_ {0.75};
  double miss_probability_ {0.45};
  double min_probability_ {0.12};
  double max_probability_ {0.97};
  double occupied_probability_ {0.5};
  double publish_rate_hz_ {2.0};
  double octomap_publish_rate_hz_ {1.0};
  double max_ray_length_m_ {300.0};
  double max_cloud_age_sec_ {0.0};
  int mapping_queue_size_ {5};
  bool remove_submap_on_disconnect_ {false};
  bool retain_submap_on_disconnect_ {true};
  double submap_timeout_sec_ {30.0};
  std::unique_ptr<octomap::OcTree> octree_;
  std::map<std::string, DroneMapping> drones_;
  std::size_t updates_since_publish_ {0};
  std::mutex map_mutex_;
  std::mutex mapping_queue_mutex_;
  std::condition_variable mapping_queue_condition_;
  std::map<std::string, SwarmMapCloud::SharedPtr> pending_map_clouds_;
  bool mapping_worker_shutdown_ {false};
  std::vector<std::thread> mapping_worker_threads_;
  std::mutex octomap_publish_mutex_;
  std::condition_variable octomap_publish_condition_;
  bool octomap_publish_requested_ {false};
  bool octomap_publish_shutdown_ {false};
  std::thread octomap_publish_thread_;
  rclcpp::Subscription<SwarmState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr global_binary_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr global_full_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr mapping_debug_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr octomap_publish_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace drone_swarm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_swarm::SwarmGlobalMapServer>());
  rclcpp::shutdown();
  return 0;
}
