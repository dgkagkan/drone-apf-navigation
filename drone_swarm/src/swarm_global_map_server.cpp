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
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
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

#include "drone_swarm/incremental_occupancy.hpp"

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
    visualization_min_z_m_ = declare_parameter<double>("visualization_min_z_m", -1.0);
    visualization_max_z_m_ = declare_parameter<double>("visualization_max_z_m", 60.0);
    if (!std::isfinite(visualization_min_z_m_) || !std::isfinite(visualization_max_z_m_) ||
      visualization_max_z_m_ <= visualization_min_z_m_)
    {
      throw std::invalid_argument("Visualization height bounds must be finite and min < max");
    }
    hit_probability_ = std::clamp(
      declare_parameter<double>("hit_probability", 0.70), 0.501, 0.999);
    miss_probability_ = std::clamp(
      declare_parameter<double>("miss_probability", 0.35), 0.001, 0.499);
    min_probability_ = std::clamp(
      declare_parameter<double>("min_probability", 0.12), 0.001, 0.499);
    max_probability_ = std::clamp(
      declare_parameter<double>("max_probability", 0.90), 0.501, 0.999);
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
    dynamic_obstacle_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("dynamic_obstacle_timeout_sec", 0.0));
    static_confirmation_sec_ = std::max(
      0.0, declare_parameter<double>("static_confirmation_sec", 8.0));
    static_confirmation_hits_ = std::max(
      1, static_cast<int>(declare_parameter<int>("static_confirmation_hits", 12)));
    fusion_config_ = {
      hit_probability_, miss_probability_, min_probability_, max_probability_,
      occupied_probability_, dynamic_obstacle_timeout_sec_, static_confirmation_sec_,
      static_cast<std::uint32_t>(static_confirmation_hits_)};
    global_fusion_ = std::make_unique<GlobalMapFusion>(fusion_config_);
    octree_ = createOctree();

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    state_sub_ = create_subscription<SwarmState>(
      "/swarm/state", state_qos,
      std::bind(&SwarmGlobalMapServer::onSwarmState, this, std::placeholders::_1));

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    global_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm/octomap_point_cloud_centers", map_qos);
    map_visualization_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/swarm/mapping_visualization", map_qos);
    global_binary_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/swarm/octomap_binary", map_qos);
    global_full_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/swarm/octomap_full", map_qos);
    mapping_debug_pub_ = create_publisher<MarkerArray>(
      "/swarm/mapping_debug", map_qos);

    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&SwarmGlobalMapServer::requestMapPublish, this));
    timeout_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SwarmGlobalMapServer::expireRetainedSubmaps, this));

    RCLCPP_INFO(
      get_logger(),
      "Dynamic global map ready: frame=%s resolution=%.2fm queue=%d publish=%.1fHz "
      "dynamic timeout=%.1fs static confirmation=%.1fs/%d hits "
      "dynamic workers octomap serialization=%.1fHz max ray=%.1fm "
      "subscriptions follow /swarm/state",
      map_frame_.c_str(), resolution_, mapping_queue_size_, publish_rate_hz_,
      dynamic_obstacle_timeout_sec_, static_confirmation_sec_, static_confirmation_hits_,
      octomap_publish_rate_hz_, max_ray_length_m_);

    octomap_publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / octomap_publish_rate_hz_),
      std::bind(&SwarmGlobalMapServer::requestOctomapPublish, this));
    octomap_publish_thread_ = std::thread(
      &SwarmGlobalMapServer::octomapPublishLoop, this);
    map_publish_thread_ = std::thread(&SwarmGlobalMapServer::mapPublishLoop, this);
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
      std::lock_guard<std::mutex> lock(map_publish_mutex_);
      map_publish_shutdown_ = true;
    }
    map_publish_condition_.notify_one();
    if (map_publish_thread_.joinable()) map_publish_thread_.join();

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
    std::uint64_t generation {0};
    std::uint64_t reset_generation {0};
    std::uint64_t revision {0};
    std::size_t color_index {0};
    // These fields belong exclusively to this drone's mapping worker.
    std::uint64_t worker_generation {std::numeric_limits<std::uint64_t>::max()};
    std::unique_ptr<octomap::OcTree> local_octree;
    std::unique_ptr<GlobalMapFusion> local_fusion;
  };

  struct PendingCloud
  {
    SwarmMapCloud::SharedPtr message;
    std::uint64_t generation {0};
  };

  // Called with map_mutex_ held. Collapse repeated global transitions until
  // the serialization worker consumes them; no octree work happens here.
  void queueGlobalChanges(const std::vector<VoxelObservation> & changes)
  {
    for (const auto & change : changes) pending_global_changes_[change.key] = change.occupied;
    if (!changes.empty()) ++global_revision_;
  }

  void resetContribution(const std::string & drone_id, DroneMapping & mapping)
  {
    ++mapping.generation;
    ++mapping.reset_generation;
    ++mapping.revision;
    ++display_revision_;
    mapping.last_stamp_ns = 0;
    local_maps_.clear(drone_id);
    // Wake the owner even without a new scan, so expired trees release memory.
    // In-flight scans from an earlier reset cannot restore their contribution.
    if (mapping.worker_started) {
      {
        std::lock_guard<std::mutex> queue_lock(mapping_queue_mutex_);
        pending_map_clouds_[drone_id] = {nullptr, mapping.generation};
      }
      mapping_queue_condition_.notify_all();
    }
  }

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
      const auto [entry, inserted] = drones_.try_emplace(drone.drone_id);
      auto & mapping = entry->second;
      if (inserted) mapping.color_index = next_color_index_++;

      if (!mapping.boot_id.empty() && !drone.boot_id.empty() &&
        mapping.boot_id != drone.boot_id)
      {
        resetContribution(drone.drone_id, mapping);
        mapping.cloud_sub.reset();
        RCLCPP_INFO(
          get_logger(), "Reset mapping contribution for %s after boot/session change",
          drone.drone_id.c_str());
      }
      if (!drone.boot_id.empty()) mapping.boot_id = drone.boot_id;
      const bool should_subscribe =
        drone.registered && drone.connected && drone.has_lidar;
      const bool was_connected = mapping.connected;
      if (was_connected != should_subscribe) {
        ++mapping.generation;
        ++display_revision_;
      }
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
          if (was_connected) resetContribution(drone.drone_id, mapping);
        }
      }
    }
  }

  void createDroneSubscription(const std::string & drone_id, DroneMapping & mapping)
  {
    const auto qos = rclcpp::SensorDataQoS().keep_last(mapping_queue_size_);
    const auto generation = mapping.generation;
    mapping.cloud_sub = create_subscription<SwarmMapCloud>(
      cloudTopic(drone_id), qos,
      [this, drone_id, generation](const SwarmMapCloud::SharedPtr message) {
        onMapCloud(drone_id, generation, message);
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

  void onMapCloud(
    const std::string & expected_drone_id, const std::uint64_t generation,
    const SwarmMapCloud::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      const auto found = drones_.find(expected_drone_id);
      if (found == drones_.end() || !found->second.connected ||
        found->second.generation != generation) return;
      const auto stamp_ns = rclcpp::Time(message->cloud.header.stamp).nanoseconds();
      if (stamp_ns > 0 && stamp_ns <= found->second.last_stamp_ns) return;
      ++found->second.received_clouds;
    }

    {
      std::lock_guard<std::mutex> lock(mapping_queue_mutex_);
      // Keep the newest scan for each drone. This prevents a slow map update
      // from building a stale queue while the live visualization keeps running.
      pending_map_clouds_[expected_drone_id] = {message, generation};
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
      PendingCloud pending;
      {
        std::unique_lock<std::mutex> lock(mapping_queue_mutex_);
        mapping_queue_condition_.wait(lock, [this, &worker_drone_id]() {
          return mapping_worker_shutdown_ ||
                 pending_map_clouds_.find(worker_drone_id) != pending_map_clouds_.end();
        });
        if (mapping_worker_shutdown_) return;

        const auto candidate = pending_map_clouds_.find(worker_drone_id);
        if (candidate == pending_map_clouds_.end()) continue;
        pending = std::move(candidate->second);
        pending_map_clouds_.erase(candidate);
      }

      try {
        processMapCloud(worker_drone_id, *mapping, pending);
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
    const PendingCloud & pending)
  {
    const auto & message = pending.message;
    std::uint64_t reset_generation = 0;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (message) {
        if (!mapping.connected || mapping.generation != pending.generation) return;
        const auto stamp_ns = rclcpp::Time(message->cloud.header.stamp).nanoseconds();
        if (stamp_ns > 0 && stamp_ns <= mapping.last_stamp_ns) return;
      }
      reset_generation = mapping.reset_generation;
    }
    if (mapping.worker_generation != reset_generation) {
      mapping.local_octree = createOctree();
      mapping.local_fusion = std::make_unique<GlobalMapFusion>(fusion_config_);
      mapping.worker_generation = reset_generation;
    }
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

    // Ray insertion never holds the metadata/global-map lock. Each worker
    // retains its own complete local OctoMap, including free-space evidence.
    const auto observation_time_ns = stamp.nanoseconds() > 0 ?
      stamp.nanoseconds() : current_time.nanoseconds();
    const auto update = insertLocalCloud(
      *mapping.local_octree, *mapping.local_fusion, scan, origin,
      effective_max_range, observation_time_ns);

    std::lock_guard<std::mutex> lock(map_mutex_);
    if (mapping.reset_generation != reset_generation) return;
    queueGlobalChanges(global_fusion_->integrateScan(
      update.free_cells, update.occupied_cells, observation_time_ns).changed_voxels);
    if (local_maps_.apply(expected_drone_id, update.local_changes)) {
      ++mapping.revision;
      ++display_revision_;
    }
    ++mapping.processed_clouds;
    mapping.last_stamp_ns = stamp.nanoseconds();
  }

  sensor_msgs::msg::PointCloud2 makeCloud(const std::vector<VoxelKey> & keys) const
  {
    const octomap::OcTree coordinates(resolution_);
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
      const auto point = coordinates.keyToCoord(toOctomapKey(key));
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

  static std::array<float, 3> droneColor(const std::size_t index)
  {
    // Spread successive registrations across red/blue/purple/cyan hues.
    // Keep the existing per-drone palette stable; no fixed fleet size.
    double hue = std::fmod(index * 0.6180339887498949, 1.0) * 0.65;
    if (hue >= 0.10) hue += 0.35;
    const double sector = hue * 6.0;
    const float fraction = static_cast<float>(sector - std::floor(sector));
    constexpr float low = 0.15F;
    const float rising = low + (1.0F - low) * fraction;
    const float falling = 1.0F - (1.0F - low) * fraction;
    switch (static_cast<int>(sector) % 6) {
      case 0: return {1.0F, rising, low};
      case 1: return {falling, 1.0F, low};
      case 2: return {low, 1.0F, rising};
      case 3: return {low, falling, 1.0F};
      case 4: return {rising, low, 1.0F};
      default: return {1.0F, low, falling};
    }
  }

  struct LocalDisplay
  {
    std::string drone_id;
    bool connected {false};
    bool publish_cloud {false};
    std::size_t color_index {0};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    std::vector<VoxelKey> keys;
  };

  MarkerArray makeDebugMarkers(const std::vector<LocalDisplay> & locals) const
  {
    const octomap::OcTree coordinates(resolution_);
    MarkerArray output;
    Marker clear;
    clear.action = Marker::DELETEALL;
    output.markers.push_back(clear);
    int marker_id = 0;
    for (const auto & local : locals) {
      if (local.keys.empty()) continue;
      Marker marker;
      marker.header.frame_id = map_frame_;
      marker.header.stamp = get_clock()->now();
      marker.ns = "mapping/" + local.drone_id;
      marker.id = marker_id++;
      marker.type = Marker::POINTS;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = static_cast<float>(resolution_ * 0.7);
      marker.scale.y = static_cast<float>(resolution_ * 0.7);
      const auto color = droneColor(local.color_index);
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = local.connected ? 0.75F : 0.35F;
      marker.points.reserve(local.keys.size());
      for (const auto & key : local.keys) {
        const auto point = coordinates.keyToCoord(toOctomapKey(key));
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

  std::array<float, 3> heightColor(const double z_m) const
  {
    // Fixed height bounds keep an existing voxel's color stable as the map grows.
    // Low to high: blue, cyan, green, yellow, red.
    const float height = 4.0F * static_cast<float>(std::clamp(
      (z_m - visualization_min_z_m_) / (visualization_max_z_m_ - visualization_min_z_m_),
      0.0, 1.0));
    return {
      std::clamp(height - 2.0F, 0.0F, 1.0F),
      std::clamp(2.0F - std::abs(height - 2.0F), 0.0F, 1.0F),
      std::clamp(2.0F - height, 0.0F, 1.0F)};
  }

  sensor_msgs::msg::PointCloud2 makeVisualization(
    const std::vector<VoxelKey> & global_keys,
    const std::vector<LocalDisplay> & locals) const
  {
    std::unordered_map<VoxelKey, std::array<float, 3>, VoxelKeyHash> local_colors;
    // Locals are ordered by drone ID. The first connected owner wins overlaps
    // deterministically, including after reconnects; geometry is always global.
    for (const auto & local : locals) {
      if (!local.connected) continue;
      const auto color = droneColor(local.color_index);
      for (const auto & key : local.keys) local_colors.emplace(key, color);
    }
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = map_frame_;
    message.header.stamp = get_clock()->now();
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(global_keys.size());
    sensor_msgs::PointCloud2Iterator<float> x(message, "x"), y(message, "y"), z(message, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> r(message, "r"), g(message, "g"),
      b(message, "b");
    const octomap::OcTree coordinates(resolution_);
    for (const auto & key : global_keys) {
      const auto point = coordinates.keyToCoord(toOctomapKey(key));
      *x = point.x();
      *y = point.y();
      *z = point.z();
      const auto color = local_colors.find(key);
      const auto rgb = color == local_colors.end() ? heightColor(point.z()) : color->second;
      *r = static_cast<std::uint8_t>(rgb[0] * 255);
      *g = static_cast<std::uint8_t>(rgb[1] * 255);
      *b = static_cast<std::uint8_t>(rgb[2] * 255);
      ++x; ++y; ++z;
      ++r; ++g; ++b;
    }
    message.is_dense = true;
    return message;
  }

  void publishMaps()
  {
    std::vector<VoxelKey> global_keys;
    std::vector<LocalDisplay> locals;
    bool publish_global = false;
    bool publish_debug = false;
    bool publish_visualization = false;
    const auto started = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      queueGlobalChanges(global_fusion_->decay(get_clock()->now().nanoseconds()).changed_voxels);
      publish_global = published_global_revision_ != global_revision_;
      const auto visualization_subscribers = map_visualization_pub_->get_subscription_count();
      publish_visualization = visualization_subscribers > 0U &&
        (publish_global || published_display_revision_ != display_revision_ ||
        visualization_subscribers != visualization_subscribers_);
      visualization_subscribers_ = visualization_subscribers;
      if (publish_global || publish_visualization) global_keys = global_fusion_->occupiedKeys();
      published_global_revision_ = global_revision_;

      const auto debug_subscribers = mapping_debug_pub_->get_subscription_count();
      publish_debug = debug_subscribers > 0U &&
        (published_display_revision_ != display_revision_ ||
        debug_subscribers != debug_subscribers_);
      debug_subscribers_ = debug_subscribers;
      published_display_revision_ = display_revision_;
      for (const auto & [drone_id, mapping] : drones_) {
        if (!mapping.debug_pub) continue;
        const auto subscribers = mapping.debug_pub->get_subscription_count();
        auto & previous = local_publications_[drone_id];
        const bool publish_local = subscribers > 0U &&
          (!previous.initialized || previous.revision != mapping.revision ||
          previous.subscribers != subscribers);
        previous = {true, mapping.revision, subscribers};
        if (publish_local || publish_debug || publish_visualization) {
          locals.push_back({drone_id, mapping.connected, publish_local,
            mapping.color_index, mapping.debug_pub, local_maps_.localKeys(drone_id)});
        }
      }
    }
    // Encoding, DDS transport and RViz debug geometry never hold map_mutex_.
    // The transient-local global publication also serves late subscribers.
    if (publish_global) global_cloud_pub_->publish(makeCloud(global_keys));
    if (publish_visualization) {
      map_visualization_pub_->publish(makeVisualization(global_keys, locals));
    }
    for (const auto & local : locals) {
      if (local.publish_cloud) local.publisher->publish(makeCloud(local.keys));
    }
    if (publish_debug) mapping_debug_pub_->publish(makeDebugMarkers(locals));
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    if (publish_global) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Global map: %zu occupied voxels; snapshot/encode/publish %.1f ms",
        global_keys.size(), elapsed_ms);
    }
  }

  void requestMapPublish()
  {
    {
      std::lock_guard<std::mutex> lock(map_publish_mutex_);
      map_publish_requested_ = true;
    }
    map_publish_condition_.notify_one();
  }

  void mapPublishLoop()
  {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(map_publish_mutex_);
        map_publish_condition_.wait(lock, [this]() {
          return map_publish_requested_ || map_publish_shutdown_;
        });
        if (map_publish_shutdown_) return;
        map_publish_requested_ = false;
      }
      try {
        publishMaps();
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(get_logger(), "Map publication failed: %s", exception.what());
        published_global_revision_ = std::numeric_limits<std::uint64_t>::max();
        published_display_revision_ = std::numeric_limits<std::uint64_t>::max();
        local_publications_.clear();
      }
    }
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

      std::unordered_map<VoxelKey, bool, VoxelKeyHash> changes;
      bool global_empty = false;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        changes.swap(pending_global_changes_);
        global_empty = global_fusion_->empty();
      }
      // This worker alone owns the global octree. Update only transitions;
      // serialization cannot delay point-cloud publication or local insertion.
      updateGlobalOccupiedTree(*octree_, changes, global_empty);
      const auto binary_subscribers = global_binary_pub_->get_subscription_count();
      const auto full_subscribers = global_full_pub_->get_subscription_count();
      if (!changes.empty() || binary_subscribers != binary_subscribers_ ||
        full_subscribers != full_subscribers_)
      {
        publishOctomapMessages(*octree_);
      }
      binary_subscribers_ = binary_subscribers;
      full_subscribers_ = full_subscribers;
    }
  }

  void publishOctomapMessages(const octomap::OcTree & snapshot)
  {
    const auto stamp = get_clock()->now();

    octomap_msgs::msg::Octomap binary;
    binary.header.frame_id = map_frame_;
    binary.header.stamp = stamp;
    if (global_binary_pub_->get_subscription_count() > 0U &&
      octomap_msgs::binaryMapToMsg(snapshot, binary))
    {
      global_binary_pub_->publish(binary);
    }

    octomap_msgs::msg::Octomap full;
    full.header = binary.header;
    if (global_full_pub_->get_subscription_count() > 0U &&
      octomap_msgs::fullMapToMsg(snapshot, full))
    {
      global_full_pub_->publish(full);
    }
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
      resetContribution(drone_id, mapping);
      mapping.disconnected_at = std::chrono::steady_clock::time_point{};
      RCLCPP_INFO(
        get_logger(), "Removed retained mapping contribution for %s after %.1fs timeout",
        drone_id.c_str(), elapsed);
    }
  }

  std::string map_frame_;
  std::string mapping_topic_prefix_;
  double resolution_ {0.5};
  double hit_probability_ {0.70};
  double visualization_min_z_m_ {-1.0};
  double visualization_max_z_m_ {60.0};
  double miss_probability_ {0.35};
  double min_probability_ {0.12};
  double max_probability_ {0.90};
  double occupied_probability_ {0.5};
  double publish_rate_hz_ {2.0};
  double octomap_publish_rate_hz_ {1.0};
  double max_ray_length_m_ {300.0};
  double max_cloud_age_sec_ {0.0};
  int mapping_queue_size_ {5};
  bool remove_submap_on_disconnect_ {false};
  bool retain_submap_on_disconnect_ {true};
  double submap_timeout_sec_ {30.0};
  double dynamic_obstacle_timeout_sec_ {0.0};
  double static_confirmation_sec_ {8.0};
  int static_confirmation_hits_ {12};
  OccupancyFusionConfig fusion_config_;
  std::unique_ptr<octomap::OcTree> octree_;
  std::unique_ptr<GlobalMapFusion> global_fusion_;
  LocalOccupiedMaps local_maps_;
  std::unordered_map<VoxelKey, bool, VoxelKeyHash> pending_global_changes_;
  std::map<std::string, DroneMapping> drones_;
  std::size_t next_color_index_ {0};
  std::uint64_t global_revision_ {0};
  std::uint64_t display_revision_ {0};
  // Publication bookkeeping belongs exclusively to its respective worker.
  struct LocalPublication
  {
    bool initialized {false};
    std::uint64_t revision {0};
    std::size_t subscribers {0};
  };
  std::map<std::string, LocalPublication> local_publications_;
  std::uint64_t published_global_revision_ {std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t published_display_revision_ {std::numeric_limits<std::uint64_t>::max()};
  std::size_t debug_subscribers_ {0};
  std::size_t visualization_subscribers_ {0};
  std::size_t binary_subscribers_ {0};
  std::size_t full_subscribers_ {0};
  std::mutex map_publish_mutex_;
  std::condition_variable map_publish_condition_;
  bool map_publish_requested_ {false};
  bool map_publish_shutdown_ {false};
  std::thread map_publish_thread_;
  std::mutex map_mutex_;
  std::mutex mapping_queue_mutex_;
  std::condition_variable mapping_queue_condition_;
  std::map<std::string, PendingCloud> pending_map_clouds_;
  bool mapping_worker_shutdown_ {false};
  std::vector<std::thread> mapping_worker_threads_;
  std::mutex octomap_publish_mutex_;
  std::condition_variable octomap_publish_condition_;
  bool octomap_publish_requested_ {false};
  bool octomap_publish_shutdown_ {false};
  std::thread octomap_publish_thread_;
  rclcpp::Subscription<SwarmState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_visualization_pub_;
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
