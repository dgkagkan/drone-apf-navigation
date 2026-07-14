// vtol_tf_broadcaster
// -------------------
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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

using std::placeholders::_1;

class VtolTfBroadcaster : public rclcpp::Node
{
public:
  VtolTfBroadcaster()
  : Node("vtol_tf_broadcaster")
  {
    map_frame_  = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // Match PX4 micro-XRCE QoS: best-effort, keep-last, volatile.
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude_v1", qos,
      std::bind(&VtolTfBroadcaster::onAttitude, this, _1));

    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&VtolTfBroadcaster::onPosition, this, _1));

    RCLCPP_INFO(
      get_logger(), "vtol_tf_broadcaster: %s -> %s (PX4 NED/FRD -> ROS ENU/FLU)",
      map_frame_.c_str(), base_frame_.c_str());
  }

private:
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
  }

  void onPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid) {
      return;  // wait until the estimator has a valid local position
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();               // ROS/sim time (use_sim_time) -> matches /scan
    t.header.frame_id = map_frame_;
    t.child_frame_id = base_frame_;

    // NED -> ENU position
    t.transform.translation.x = msg->y;   // East  = NED y
    t.transform.translation.y = msg->x;   // North = NED x
    t.transform.translation.z = -msg->z;  // Up    = -NED z

    tf2::Quaternion q;
    if (have_att_) {
      q = q_enu_;                          // full attitude
    } else {
      // Fallback: yaw only from heading (NED yaw -> ENU yaw = π/2 - heading)
      q.setRPY(0.0, 0.0, M_PI_2 - static_cast<double>(msg->heading));
    }
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::string map_frame_;
  std::string base_frame_;
  bool have_att_ {false};
  tf2::Quaternion q_enu_ {0.0, 0.0, 0.0, 1.0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr att_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VtolTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
