// mission_manager
// ---------------
// Small APF mission node for SITL: attractive velocity toward a local ENU goal,
// plus repulsive velocity from nearby lidar points. Output is a ROS ENU
// TwistStamped consumed by vtol_offboard_control.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager()
  : Node("mission_manager")
  {
    goal_x_ = declare_parameter<double>("goal_x", 25.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_z_ = declare_parameter<double>("goal_z", 5.0);
    cruise_speed_ = declare_parameter<double>("cruise_speed", 3.0);
    max_vertical_speed_ = declare_parameter<double>("max_vertical_speed", 1.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 1.5);
    attractive_gain_ = declare_parameter<double>("attractive_gain", 0.7);
    repulsive_gain_ = declare_parameter<double>("repulsive_gain", 1.4);
    obstacle_influence_radius_ = declare_parameter<double>("obstacle_influence_radius", 8.0);
    obstacle_min_height_ = declare_parameter<double>("obstacle_min_height", -1.0);
    obstacle_max_height_ = declare_parameter<double>("obstacle_max_height", 2.0);
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/scan_3d/points");

    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos,
      std::bind(&MissionManager::onLocalPosition, this, _1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MissionManager::onCloud, this, _1));

    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/apf/velocity_setpoint", 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("/apf/goal_reached", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MissionManager::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "mission_manager APF goal ENU=(%.1f, %.1f, %.1f), cloud=%s",
      goal_x_, goal_y_, goal_z_, cloud_topic_.c_str());
  }

private:
  struct Vec3
  {
    double x {0.0};
    double y {0.0};
    double z {0.0};
  };

  static double clampNorm2d(Vec3 & v, const double max_norm)
  {
    const double norm = std::hypot(v.x, v.y);
    if (norm > max_norm && norm > 1e-6) {
      const double scale = max_norm / norm;
      v.x *= scale;
      v.y *= scale;
    }
    return norm;
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid) {
      have_position_ = false;
      return;
    }

    position_enu_.x = msg->y;
    position_enu_.y = msg->x;
    position_enu_.z = -msg->z;
    heading_ned_ = msg->heading;
    have_position_ = true;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_repulsion_body_ = computeRepulsionBody(*msg);
    have_cloud_ = true;
  }

  Vec3 computeRepulsionBody(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    Vec3 repulsion;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < obstacle_min_height_ || z > obstacle_max_height_) {
        continue;
      }

      const double d = std::hypot(x, y);
      if (d < 1e-3 || d > obstacle_influence_radius_) {
        continue;
      }

      const double strength = repulsive_gain_ *
        (1.0 / d - 1.0 / obstacle_influence_radius_) / (d * d);

      repulsion.x += -x / d * strength;
      repulsion.y += -y / d * strength;
    }

    clampNorm2d(repulsion, cruise_speed_);
    return repulsion;
  }

  Vec3 bodyFluToEnu(const Vec3 & body) const
  {
    // heading is PX4 NED yaw from North. Convert body FLU x/y to ENU x/y.
    const double yaw_enu = M_PI_2 - heading_ned_;
    const double c = std::cos(yaw_enu);
    const double s = std::sin(yaw_enu);

    return Vec3{
      c * body.x - s * body.y,
      s * body.x + c * body.y,
      body.z};
  }

  void onTimer()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "map";

    std_msgs::msg::Bool reached_msg;
    reached_msg.data = false;

    if (!have_position_) {
      velocity_pub_->publish(cmd);
      reached_pub_->publish(reached_msg);
      return;
    }

    Vec3 to_goal{
      goal_x_ - position_enu_.x,
      goal_y_ - position_enu_.y,
      goal_z_ - position_enu_.z};

    const double horizontal_distance = std::hypot(to_goal.x, to_goal.y);
    const double vertical_error = to_goal.z;
    if (horizontal_distance < goal_tolerance_ && std::abs(vertical_error) < goal_tolerance_) {
      reached_msg.data = true;
      velocity_pub_->publish(cmd);
      reached_pub_->publish(reached_msg);
      return;
    }

    Vec3 attractive{
      attractive_gain_ * to_goal.x,
      attractive_gain_ * to_goal.y,
      0.0};
    clampNorm2d(attractive, cruise_speed_);

    Vec3 repulsion_enu;
    if (have_cloud_) {
      repulsion_enu = bodyFluToEnu(latest_repulsion_body_);
    }

    Vec3 velocity{
      attractive.x + repulsion_enu.x,
      attractive.y + repulsion_enu.y,
      std::clamp(attractive_gain_ * vertical_error, -max_vertical_speed_, max_vertical_speed_)};

    clampNorm2d(velocity, cruise_speed_);

    cmd.twist.linear.x = velocity.x;
    cmd.twist.linear.y = velocity.y;
    cmd.twist.linear.z = velocity.z;
    velocity_pub_->publish(cmd);
    reached_pub_->publish(reached_msg);
  }

  std::string cloud_topic_;
  double goal_x_ {25.0};
  double goal_y_ {0.0};
  double goal_z_ {5.0};
  double cruise_speed_ {3.0};
  double max_vertical_speed_ {1.0};
  double goal_tolerance_ {1.5};
  double attractive_gain_ {0.7};
  double repulsive_gain_ {1.4};
  double obstacle_influence_radius_ {8.0};
  double obstacle_min_height_ {-1.0};
  double obstacle_max_height_ {2.0};
  double heading_ned_ {0.0};
  bool have_position_ {false};
  bool have_cloud_ {false};
  Vec3 position_enu_;
  Vec3 latest_repulsion_body_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
