#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;

class LidarProcessorNode : public rclcpp::Node
{
public:
  LidarProcessorNode()
  : Node("lidar_processor"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_3d/points");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/perception/obstacles");
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    min_range_m_ = std::max(0.0, declare_parameter<double>("min_range_m", 0.3));
    max_range_m_ = std::max(
      min_range_m_, declare_parameter<double>("max_range_m", 70.0));
    min_commanded_range_m_ = std::max(
      min_range_m_, declare_parameter<double>("commanded_range.min_m", 70.0));
    max_commanded_range_m_ = std::max(
      min_commanded_range_m_, declare_parameter<double>("commanded_range.max_m", 300.0));
    ground_height_m_ = declare_parameter<double>("ground_height_m", 0.4);
    voxel_size_m_ = std::max(0.0, declare_parameter<double>("voxel_size_m", 0.2));

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos, std::bind(&LidarProcessorNode::onCloud, this, _1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    range_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/perception/lidar_range_override",
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LidarProcessorNode::onRangeCommand, this, _1));

    RCLCPP_INFO(
      get_logger(), "Lidar processor: %s -> %s in %s, range %.1f-%.1fm, voxel %.2fm",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str(),
      min_range_m_, max_range_m_, voxel_size_m_);
  }

private:
  void onRangeCommand(const std_msgs::msg::Float64::SharedPtr message)
  {
    if (!std::isfinite(message->data) || message->data < min_commanded_range_m_ ||
      message->data > max_commanded_range_m_)
    {
      RCLCPP_WARN(
        get_logger(), "Rejected LiDAR range %.1f m; expected %.1f-%.1f m",
        message->data, min_commanded_range_m_, max_commanded_range_m_);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(range_mutex_);
      max_range_m_ = message->data;
    }
    RCLCPP_INFO(get_logger(), "Active LiDAR obstacle range set to %.1f m", message->data);
  }

  struct VoxelKey
  {
    int64_t x;
    int64_t y;
    int64_t z;

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      const auto h1 = std::hash<int64_t>{}(key.x);
      const auto h2 = std::hash<int64_t>{}(key.y);
      const auto h3 = std::hash<int64_t>{}(key.z);
      return h1 ^ (h2 << 1U) ^ (h3 << 2U);
    }
  };

  static void rotateAndTranslate(
    const geometry_msgs::msg::Transform & transform,
    double x, double y, double z,
    double & output_x, double & output_y, double & output_z)
  {
    const auto & q = transform.rotation;
    const double tx = 2.0 * (q.y * z - q.z * y);
    const double ty = 2.0 * (q.z * x - q.x * z);
    const double tz = 2.0 * (q.x * y - q.y * x);
    output_x = x + q.w * tx + (q.y * tz - q.z * ty) + transform.translation.x;
    output_y = y + q.w * ty + (q.z * tx - q.x * tz) + transform.translation.y;
    output_z = z + q.w * tz + (q.x * ty - q.y * tx) + transform.translation.z;
  }

  VoxelKey voxelKey(double x, double y, double z) const
  {
    return {
      static_cast<int64_t>(std::floor(x / voxel_size_m_)),
      static_cast<int64_t>(std::floor(y / voxel_size_m_)),
      static_cast<int64_t>(std::floor(z / voxel_size_m_))};
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LiDAR cloud has no frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        output_frame_, cloud->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "LiDAR TF unavailable: %s", error.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud->header;
    output.header.frame_id = output_frame_;
    output.height = 1;
    output.is_bigendian = false;
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cloud->width * cloud->height);
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");

    std::unordered_set<VoxelKey, VoxelHash> occupied_voxels;
    double max_range_m = 0.0;
    {
      std::lock_guard<std::mutex> lock(range_mutex_);
      max_range_m = max_range_m_;
    }
    const double min_range_squared = min_range_m_ * min_range_m_;
    const double max_range_squared = max_range_m * max_range_m;
    std::size_t kept = 0;
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      const double range_squared = *x * *x + *y * *y + *z * *z;
      if (!std::isfinite(range_squared) || range_squared < min_range_squared ||
        range_squared >= max_range_squared)
      {
        continue;
      }

      double map_x = 0.0;
      double map_y = 0.0;
      double map_z = 0.0;
      rotateAndTranslate(transform.transform, *x, *y, *z, map_x, map_y, map_z);
      if (!std::isfinite(map_x) || !std::isfinite(map_y) || !std::isfinite(map_z) ||
        map_z <= ground_height_m_)
      {
        continue;
      }
      if (voxel_size_m_ > 0.0 && !occupied_voxels.insert(voxelKey(map_x, map_y, map_z)).second) {
        continue;
      }

      *output_x = static_cast<float>(map_x);
      *output_y = static_cast<float>(map_y);
      *output_z = static_cast<float>(map_z);
      ++output_x;
      ++output_y;
      ++output_z;
      ++kept;
    }

    modifier.resize(kept);
    cloud_pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double min_range_m_ {0.3};
  std::mutex range_mutex_;
  double max_range_m_ {70.0};
  double min_commanded_range_m_ {70.0};
  double max_commanded_range_m_ {300.0};
  double ground_height_m_ {0.4};
  double voxel_size_m_ {0.2};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr range_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
