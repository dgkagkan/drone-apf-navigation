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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using std::placeholders::_1;

class PointCloudThrottle : public rclcpp::Node
{
public:
  PointCloudThrottle()
  : Node("point_cloud_throttle")
  {
    const double publish_rate = std::clamp(
      declare_parameter<double>("publish_rate", 1.0), 0.1, 10.0);
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_3d/points");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/scan_3d/filtered_points");
    min_valid_range_ = std::max(
      0.0, declare_parameter<double>("min_valid_range", 0.2));
    max_valid_range_ = std::max(
      min_valid_range_, declare_parameter<double>("max_valid_range", 78.0));

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&PointCloudThrottle::onCloud, this, _1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&PointCloudThrottle::publishLatest, this));

    RCLCPP_INFO(
      get_logger(), "PointCloud filter: %s -> %s at %.1fHz, valid range %.1f-%.1fm",
      input_topic_.c_str(), output_topic_.c_str(), publish_rate,
      min_valid_range_, max_valid_range_);
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
    if (filterCloud(*cloud, filtered)) cloud_pub_->publish(filtered);
  }

  bool filterCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output)
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
        if (!std::isfinite(range_squared) || range_squared < min_range_squared ||
          range_squared >= max_range_squared)
        {
          ++rejected;
          continue;
        }

        output.data.insert(output.data.end(), point, point + input.point_step);
      }
    }

    output.width = output.data.size() / input.point_step;
    output.row_step = output.width * output.point_step;
    output.is_dense = true;

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
  double min_valid_range_{0.2};
  double max_valid_range_{78.0};
  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudThrottle>());
  rclcpp::shutdown();
  return 0;
}
