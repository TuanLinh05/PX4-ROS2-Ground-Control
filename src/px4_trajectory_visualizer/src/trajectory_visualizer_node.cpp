#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

static constexpr size_t MAX_POINTS = 2000;

class TrajectoryVisualizerNode : public rclcpp::Node
{
public:
  TrajectoryVisualizerNode() : Node("trajectory_visualizer_node")
  {
    rclcpp::QoS px4_qos(5);
    px4_qos.best_effort().durability_volatile();

    // Subscribe to vehicle local position
    sub_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        handlePosition(msg);
      });

    // Subscribe to trajectory setpoint (for setpoint marker)
    sub_sp_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos,
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        handleSetpoint(msg);
      });

    // Publishers
    pub_path_      = create_publisher<nav_msgs::msg::Path>("/trajectory/path", 10);
    pub_setpoint_  = create_publisher<visualization_msgs::msg::Marker>("/visualization/setpoint", 10);
    pub_waypoint_  = create_publisher<visualization_msgs::msg::Marker>("/visualization/waypoint", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "TrajectoryVisualizer started. Topics: /trajectory/path, /visualization/*");
  }

private:
  void handlePosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr & msg)
  {
    auto now = get_clock()->now();

    // NED → ENU: x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned
    double ex = msg->y;
    double ey = msg->x;
    double ez = -msg->z;

    double yaw_enu = M_PI_2 - msg->heading;

    // ── Path ────────────────────────────────────────────────
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = ex;
    pose.pose.position.y = ey;
    pose.pose.position.z = ez;
    pose.pose.orientation.w = std::cos(yaw_enu / 2.0);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw_enu / 2.0);

    path_.poses.push_back(pose);
    if (path_.poses.size() > MAX_POINTS)
      path_.poses.erase(path_.poses.begin());
    path_.header = pose.header;
    pub_path_->publish(path_);

    // ── TF broadcast ────────────────────────────────────────
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = now;
    t.header.frame_id = "map";
    t.child_frame_id  = "base_link";
    t.transform.translation.x = ex;
    t.transform.translation.y = ey;
    t.transform.translation.z = ez;
    t.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  void handleSetpoint(const px4_msgs::msg::TrajectorySetpoint::SharedPtr & msg)
  {
    auto now = get_clock()->now();

    // NED → ENU
    double ex = msg->position[1];
    double ey = msg->position[0];
    double ez = -msg->position[2];

    // Check for NaN
    if (std::isnan(ex) || std::isnan(ey) || std::isnan(ez)) return;

    // ── Setpoint marker (cyan diamond) ──────────────────────
    visualization_msgs::msg::Marker sp;
    sp.header.stamp    = now;
    sp.header.frame_id = "map";
    sp.ns = "setpoint";
    sp.id = 0;
    sp.type   = visualization_msgs::msg::Marker::CUBE;
    sp.action = visualization_msgs::msg::Marker::ADD;
    sp.pose.position.x = ex;
    sp.pose.position.y = ey;
    sp.pose.position.z = ez;
    // Rotate 45 deg for diamond look
    sp.pose.orientation.w = 0.924;
    sp.pose.orientation.x = 0.0;
    sp.pose.orientation.y = 0.0;
    sp.pose.orientation.z = 0.383;
    sp.scale.x = 0.2;
    sp.scale.y = 0.2;
    sp.scale.z = 0.2;
    sp.color.r = 0.0f;
    sp.color.g = 0.83f;
    sp.color.b = 0.67f;
    sp.color.a = 0.9f;
    sp.lifetime = rclcpp::Duration::from_seconds(0.5);
    pub_setpoint_->publish(sp);

    // ── Setpoint path (dashed cyan line via LINE_STRIP) ─────
    setpoint_path_.push_back({ex, ey, ez});
    if (setpoint_path_.size() > MAX_POINTS)
      setpoint_path_.erase(setpoint_path_.begin());

    visualization_msgs::msg::Marker sp_trail;
    sp_trail.header.stamp    = now;
    sp_trail.header.frame_id = "map";
    sp_trail.ns = "setpoint_trail";
    sp_trail.id = 1;
    sp_trail.type   = visualization_msgs::msg::Marker::LINE_STRIP;
    sp_trail.action = visualization_msgs::msg::Marker::ADD;
    sp_trail.pose.orientation.w = 1.0;
    sp_trail.scale.x = 0.02;
    sp_trail.color.r = 0.0f;
    sp_trail.color.g = 0.83f;
    sp_trail.color.b = 0.67f;
    sp_trail.color.a = 0.5f;

    for (const auto & pt : setpoint_path_) {
      geometry_msgs::msg::Point p;
      p.x = pt[0]; p.y = pt[1]; p.z = pt[2];
      sp_trail.points.push_back(p);
    }
    pub_setpoint_->publish(sp_trail);
  }

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_pos_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_sp_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_waypoint_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_;
  std::vector<std::array<double, 3>> setpoint_path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
