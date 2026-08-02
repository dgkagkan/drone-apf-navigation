// vehicle_tf_broadcaster_node
// ---------------------------
// Broadcasts the TF  map -> base_link  from the PX4 estimator so RViz has a
// full TF chain (robot_state_publisher supplies base_link -> lidar_link from
// the URDF).
//
// PX4 publishes in NED (x=North, y=East, z=Down) + body FRD; ROS/RViz use
// ENU (x=East, y=North, z=Up) + body FLU. We convert both position and
// orientation. The orientation conversion is the standard px4_ros_com one:
//     q_enu_flu = NED_ENU * q_px4(frd->ned) * FRD_FLU
//
// PX4 micro-XRCE topics are best_effort + versioned ("_v1" suffix on main),
// otherwise the subscription sees 0 publishers.

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

using std::placeholders::_1;

class VehicleTfBroadcasterNode : public rclcpp::Node
{
public:
  VehicleTfBroadcasterNode()
  : Node("vehicle_tf_broadcaster")
  {
    map_frame_  = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    use_ground_truth_ = declare_parameter<bool>("use_ground_truth", true);
    const auto ground_truth_topic = declare_parameter<std::string>(
      "ground_truth_topic", "/drone/ground_truth/odometry");

    // Match PX4 micro-XRCE QoS: best-effort, keep-last, volatile.
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (use_ground_truth_) {
      ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        ground_truth_topic, rclcpp::SensorDataQoS(),
        std::bind(&VehicleTfBroadcasterNode::onGroundTruth, this, _1));
    } else {
      att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude_v1", qos,
        std::bind(&VehicleTfBroadcasterNode::onAttitude, this, _1));

      pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", qos,
        std::bind(&VehicleTfBroadcasterNode::onPosition, this, _1));
    }

    RCLCPP_INFO(
      get_logger(), "Vehicle TF: %s -> %s from %s",
      map_frame_.c_str(), base_frame_.c_str(),
      use_ground_truth_ ? "Gazebo ground truth" : "PX4 estimator");
  }

private:
  void onGroundTruth(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & position = msg->pose.pose.position;
    const auto & orientation = msg->pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(quaternion_norm) ||
      quaternion_norm < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring invalid Gazebo odometry pose");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = position.x;
    transform.transform.translation.y = position.y;
    transform.transform.translation.z = position.z;
    transform.transform.rotation.x = orientation.x / quaternion_norm;
    transform.transform.rotation.y = orientation.y / quaternion_norm;
    transform.transform.rotation.z = orientation.z / quaternion_norm;
    transform.transform.rotation.w = orientation.w / quaternion_norm;
    tf_broadcaster_->sendTransform(transform);
  }

  // Cache the latest ENU/FLU orientation from vehicle_attitude.
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    // PX4 q = [w, x, y, z], rotation body-FRD -> NED. tf2 ctor is (x, y, z, w).
    const tf2::Quaternion q_px4(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);

    // Static frame conversions (px4_ros_com constants).
    // NED_ENU : (w,x,y,z) = (0, √2/2, √2/2, 0)
    // FRD_FLU : (w,x,y,z) = (0, 1, 0, 0)  -> 180° about body X
    static const tf2::Quaternion NED_ENU(0.7071067811865476, 0.7071067811865476, 0.0, 0.0);
    static const tf2::Quaternion FRD_FLU(1.0, 0.0, 0.0, 0.0);

    tf2::Quaternion q_enu = NED_ENU * q_px4 * FRD_FLU;
    q_enu.normalize();
    q_enu_ = q_enu;
    have_att_ = true;

    // Attitude is updated faster than local position. Publishing here keeps
    // LiDAR-to-map transforms aligned during turns instead of waiting for the
    // next position sample and painting rotated obstacle ghosts into OctoMap.
    publishTransform(now());
  }

  void onPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid) {
      return;  // wait until the estimator has a valid local position
    }

    // NED -> ENU position
    east_ = msg->y;
    north_ = msg->x;
    altitude_ = -msg->z;
    heading_ned_ = msg->heading;
    have_position_ = true;

    publishTransform(now());
  }

  void publishTransform(const rclcpp::Time & stamp)
  {
    if (!have_position_) {
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;               // ROS/sim time matches the LiDAR cloud clock
    t.header.frame_id = map_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = east_;
    t.transform.translation.y = north_;
    t.transform.translation.z = altitude_;

    tf2::Quaternion q;
    if (have_att_) {
      q = q_enu_;                          // full attitude
    } else {
      // Fallback: yaw only from heading (NED yaw -> ENU yaw = π/2 - heading)
      q.setRPY(0.0, 0.0, M_PI_2 - heading_ned_);
    }
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::string map_frame_;
  std::string base_frame_;
  bool use_ground_truth_ {true};
  bool have_att_ {false};
  bool have_position_ {false};
  double east_ {0.0};
  double north_ {0.0};
  double altitude_ {0.0};
  double heading_ned_ {0.0};
  tf2::Quaternion q_enu_ {0.0, 0.0, 0.0, 1.0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
