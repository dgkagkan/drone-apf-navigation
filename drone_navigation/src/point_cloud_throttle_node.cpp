// Low-rate PointCloud2 relay for visualization-only mapping workloads.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <drone_interfaces/msg/swarm_map_cloud.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using std::placeholders::_1;

class PointCloudThrottle : public rclcpp::Node
{
public:
  PointCloudThrottle()
  : Node("point_cloud_throttle"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const double publish_rate = std::clamp(
      declare_parameter<double>("publish_rate", 1.0), 0.1, 10.0);
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_3d/points");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/scan_3d/filtered_points");
    secondary_output_topic_ =
      declare_parameter<std::string>("secondary_output_topic", "");
    drone_id_ = declare_parameter<std::string>("drone_id", "");
    mapping_frame_ = declare_parameter<std::string>("mapping_frame", "");
    map_cloud_topic_ = declare_parameter<std::string>("map_cloud_topic", "");
    mapping_tf_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("mapping_tf_timeout_sec", 0.05));
    mapping_queue_size_ = std::clamp(
      static_cast<int>(declare_parameter<int>("mapping_queue_size", 5)), 1, 20);
    secondary_include_max_range_rays_ = declare_parameter<bool>(
      "secondary_include_max_range_rays", false);
    min_valid_range_ = std::max(
      0.0, declare_parameter<double>("min_valid_range", 0.2));
    max_valid_range_ = std::max(
      min_valid_range_, declare_parameter<double>("max_valid_range", 200.0));

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&PointCloudThrottle::onCloud, this, _1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    if (!secondary_output_topic_.empty()) {
      secondary_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        secondary_output_topic_, qos);
    }
    if (!map_cloud_topic_.empty()) {
      map_cloud_pub_ = create_publisher<drone_interfaces::msg::SwarmMapCloud>(
        map_cloud_topic_, rclcpp::SensorDataQoS().keep_last(mapping_queue_size_));
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&PointCloudThrottle::publishLatest, this));

    RCLCPP_INFO(
      get_logger(), "PointCloud filter: %s -> %s%s%s%s at %.1fHz, valid range %.1f-%.1fm",
      input_topic_.c_str(), output_topic_.c_str(),
      secondary_output_topic_.empty() ? "" : " and ",
      secondary_output_topic_.c_str(),
      map_cloud_topic_.empty() ? "" : " plus timestamped map cloud",
      publish_rate,
      min_valid_range_, max_valid_range_);
    if (!map_cloud_topic_.empty() && (drone_id_.empty() || mapping_frame_.empty())) {
      RCLCPP_WARN(
        get_logger(),
        "map_cloud_topic is set but drone_id or mapping_frame is empty; map clouds are disabled");
      map_cloud_pub_.reset();
    }
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = cloud;
  }

  void publishLatest()
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
    }
    if (!cloud) return;

    sensor_msgs::msg::PointCloud2 filtered;
    sensor_msgs::msg::PointCloud2 mapping_cloud;
    if (!filterCloud(
        *cloud, filtered,
        secondary_include_max_range_rays_ ? &mapping_cloud : nullptr))
    {
      return;
    }

    cloud_pub_->publish(filtered);
    if (secondary_cloud_pub_) {
      secondary_cloud_pub_->publish(
        secondary_include_max_range_rays_ ? mapping_cloud : filtered);
    }
    if (map_cloud_pub_) {
      publishMapCloud(
        *cloud,
        secondary_include_max_range_rays_ ? mapping_cloud : filtered);
    }
  }

  void publishMapCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    const sensor_msgs::msg::PointCloud2 & mapping_cloud)
  {
    if (input.header.stamp.sec == 0 && input.header.stamp.nanosec == 0) {
      ++mapping_dropped_no_tf_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping map cloud from %s because it has no timestamp",
        drone_id_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        mapping_frame_, input.header.frame_id, rclcpp::Time(input.header.stamp),
        tf2::durationFromSec(mapping_tf_timeout_sec_));
    } catch (const tf2::TransformException & exception) {
      ++mapping_dropped_no_tf_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping map cloud from %s: TF %s <- %s at cloud timestamp unavailable "
        "(%s); dropped=%zu",
        drone_id_.c_str(), mapping_frame_.c_str(), input.header.frame_id.c_str(),
        exception.what(), mapping_dropped_no_tf_);
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      tf2::doTransform(mapping_cloud, transformed, transform);
    } catch (const tf2::TransformException & exception) {
      ++mapping_dropped_no_tf_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping map cloud from %s after TF conversion failed: %s",
        drone_id_.c_str(), exception.what());
      return;
    }
    transformed.header.stamp = input.header.stamp;
    transformed.header.frame_id = mapping_frame_;

    drone_interfaces::msg::SwarmMapCloud message;
    message.header = transformed.header;
    message.drone_id = drone_id_;
    message.cloud = std::move(transformed);
    message.sensor_origin.x = transform.transform.translation.x;
    message.sensor_origin.y = transform.transform.translation.y;
    message.sensor_origin.z = transform.transform.translation.z;
    message.max_range_m = max_valid_range_;
    map_cloud_pub_->publish(std::move(message));
    ++mapping_transformed_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Mapping relay %s: transformed=%zu dropped_no_tf=%zu queue=%d",
      drone_id_.c_str(), mapping_transformed_, mapping_dropped_no_tf_,
      mapping_queue_size_);
  }

  bool filterCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output,
    sensor_msgs::msg::PointCloud2 * mapping_output)
  {
    if (input.is_bigendian) {
      RCLCPP_WARN_ONCE(get_logger(), "Big-endian PointCloud2 is not supported");
      return false;
    }

    const auto * x_field = findField(input, "x");
    const auto * y_field = findField(input, "y");
    const auto * z_field = findField(input, "z");
    if (!x_field || !y_field || !z_field || input.point_step == 0) {
      RCLCPP_WARN_ONCE(get_logger(), "PointCloud2 does not contain usable x/y/z fields");
      return false;
    }

    output = input;
    output.height = 1;
    output.width = 0;
    output.row_step = 0;
    output.data.clear();
    output.data.reserve(input.width * input.height * input.point_step);
    if (mapping_output) {
      *mapping_output = output;
      mapping_output->data.reserve(input.width * input.height * input.point_step);
    }

    const double min_range_squared = min_valid_range_ * min_valid_range_;
    const double max_range_squared = max_valid_range_ * max_valid_range_;
    std::size_t rejected = 0;

    for (std::size_t row = 0; row < input.height; ++row) {
      for (std::size_t column = 0; column < input.width; ++column) {
        const std::size_t offset = row * input.row_step + column * input.point_step;
        if (offset + input.point_step > input.data.size()) {
          ++rejected;
          continue;
        }

        const auto * point = input.data.data() + offset;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!readCoordinate(point, input.point_step, *x_field, x) ||
          !readCoordinate(point, input.point_step, *y_field, y) ||
          !readCoordinate(point, input.point_step, *z_field, z))
        {
          ++rejected;
          continue;
        }

        const double range_squared = x * x + y * y + z * z;
        if (!std::isfinite(range_squared) || range_squared < min_range_squared)
        {
          ++rejected;
          continue;
        }

        if (range_squared < max_range_squared) {
          output.data.insert(output.data.end(), point, point + input.point_step);
          if (mapping_output) {
            mapping_output->data.insert(
              mapping_output->data.end(), point, point + input.point_step);
          }
        } else if (mapping_output) {
          // OctoMap interprets an endpoint beyond sensor_model.max_range as a
          // free-space ray without marking that endpoint occupied. Keeping
          // these rays lets a later observation clear a removed obstacle.
          mapping_output->data.insert(
            mapping_output->data.end(), point, point + input.point_step);
        } else {
          ++rejected;
        }
      }
    }

    output.width = output.data.size() / input.point_step;
    output.row_step = output.width * output.point_step;
    output.is_dense = true;
    if (mapping_output) {
      mapping_output->width = mapping_output->data.size() / input.point_step;
      mapping_output->row_step = mapping_output->width * mapping_output->point_step;
      mapping_output->is_dense = true;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Filtered cloud: kept %u points, rejected %zu max-range/noise points",
      output.width, rejected);
    return true;
  }

  const sensor_msgs::msg::PointField * findField(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & name) const
  {
    const auto field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [&name](const auto & candidate) {return candidate.name == name;});
    return field == cloud.fields.end() ? nullptr : &(*field);
  }

  bool readCoordinate(
    const uint8_t * point,
    const std::size_t point_step,
    const sensor_msgs::msg::PointField & field,
    double & value) const
  {
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      if (field.offset + sizeof(float) > point_step) return false;
      float coordinate = 0.0F;
      std::memcpy(&coordinate, point + field.offset, sizeof(coordinate));
      value = coordinate;
      return true;
    }
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
      if (field.offset + sizeof(double) > point_step) return false;
      std::memcpy(&value, point + field.offset, sizeof(value));
      return true;
    }
    return false;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string secondary_output_topic_;
  std::string drone_id_;
  std::string mapping_frame_;
  std::string map_cloud_topic_;
  bool secondary_include_max_range_rays_{false};
  double min_valid_range_{0.2};
  double max_valid_range_{200.0};
  double mapping_tf_timeout_sec_{0.05};
  int mapping_queue_size_{5};
  std::size_t mapping_dropped_no_tf_{0};
  std::size_t mapping_transformed_{0};
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr secondary_cloud_pub_;
  rclcpp::Publisher<drone_interfaces::msg::SwarmMapCloud>::SharedPtr map_cloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudThrottle>());
  rclcpp::shutdown();
  return 0;
}
