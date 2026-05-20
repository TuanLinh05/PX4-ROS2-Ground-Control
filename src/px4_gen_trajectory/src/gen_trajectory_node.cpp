#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cmath>

// Threshold (m) để coi là đã tới waypoint
static constexpr double DEFAULT_ARRIVE_DIST = 0.4;

// Default: hover tại vị trí ban đầu, cao 3m — chờ GCS upload waypoints
static const std::vector<std::array<double, 3>> DEFAULT_WAYPOINTS = {
  { 0.0,  0.0, -3.0},   // Hover at origin
};

class GenTrajectoryNode : public rclcpp::Node
{
public:
  GenTrajectoryNode() : Node("gen_trajectory_node")
  {
    // ── Parameters ─────────────────────────────────────────────────
    declare_parameter("arrive_distance", DEFAULT_ARRIVE_DIST);
    arrive_dist_ = get_parameter("arrive_distance").as_double();

    rclcpp::QoS px4_qos(5);
    px4_qos.best_effort().durability_volatile();

    // ── Subscribers ────────────────────────────────────────────────
    sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        pos_ = {msg->x, msg->y, msg->z};
        got_pos_ = true;
        checkAndAdvance();
      });

    // Subscribe to external waypoints from GCS
    // Format: flat array [x1,y1,z1, x2,y2,z2, ...]
    sub_waypoints_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/trajectory/set_waypoints", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        setExternalWaypoints(msg->data);
      });

    // Subscribe to reset command
    sub_reset_ = create_subscription<std_msgs::msg::Bool>(
      "/trajectory/reset", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          wp_idx_ = 0;
          RCLCPP_INFO(get_logger(), "[TRAJ] Reset to WP0");
          publishCurrentWaypoint();
        }
      });

    // ── Publishers ─────────────────────────────────────────────────
    pub_waypoint_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/waypoint/current", 10);

    // Publish trajectory status for GCS
    pub_status_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/trajectory/status", 10);

    // Status timer — publish trajectory status at 2 Hz
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500),
      [this]() { publishStatus(); });

    // Load default hover waypoint — drone hovers until GCS uploads trajectory
    waypoints_ = DEFAULT_WAYPOINTS;

    // Publish waypoint đầu tiên ngay khi khởi động
    publishCurrentWaypoint();
    RCLCPP_INFO(get_logger(), "[TRAJ] Hovering at (0, 0, -3.0). Waiting for GCS to upload trajectory...");
  }

private:
  void setExternalWaypoints(const std::vector<double> & data)
  {
    if (data.size() < 3 || data.size() % 3 != 0) {
      RCLCPP_WARN(get_logger(), "[TRAJ] Invalid waypoint data: size=%zu (must be multiple of 3)", data.size());
      return;
    }

    waypoints_.clear();
    for (size_t i = 0; i + 2 < data.size(); i += 3) {
      waypoints_.push_back({data[i], data[i+1], data[i+2]});
    }
    wp_idx_ = 0;

    RCLCPP_INFO(get_logger(), "[TRAJ] Loaded %zu waypoints from GCS.", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "[TRAJ]   WP%zu: (%.2f, %.2f, %.2f)",
        i, waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
    }

    publishCurrentWaypoint();
  }

  void checkAndAdvance()
  {
    if (!got_pos_ || wp_idx_ >= waypoints_.size()) return;

    const auto & wp = waypoints_[wp_idx_];
    double dx = pos_[0] - wp[0];
    double dy = pos_[1] - wp[1];
    double dz = pos_[2] - wp[2];
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < arrive_dist_) {
      wp_idx_++;
      if (wp_idx_ >= waypoints_.size()) {
        RCLCPP_INFO(get_logger(), "[TRAJ] Hoan thanh quy dao! (%zu waypoints)", waypoints_.size());
        return;
      }
      RCLCPP_INFO(get_logger(), "[TRAJ] Den WP%zu. Chuyen sang WP%zu: (%.1f, %.1f, %.1f)",
        wp_idx_ - 1, wp_idx_,
        waypoints_[wp_idx_][0],
        waypoints_[wp_idx_][1],
        waypoints_[wp_idx_][2]);
      publishCurrentWaypoint();
    }
  }

  void publishCurrentWaypoint()
  {
    if (wp_idx_ >= waypoints_.size()) return;
    const auto & wp = waypoints_[wp_idx_];

    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp    = get_clock()->now();
    msg.header.frame_id = "map";
    msg.point.x = wp[0];
    msg.point.y = wp[1];
    msg.point.z = wp[2];
    pub_waypoint_->publish(msg);
  }

  void publishStatus()
  {
    // Status format: [current_wp_idx, total_wp_count, current_wp_x, current_wp_y, current_wp_z, done]
    std_msgs::msg::Float64MultiArray status{};
    double done = (wp_idx_ >= waypoints_.size()) ? 1.0 : 0.0;
    if (!waypoints_.empty() && wp_idx_ < waypoints_.size()) {
      const auto & wp = waypoints_[wp_idx_];
      status.data = {
        double(wp_idx_), double(waypoints_.size()),
        wp[0], wp[1], wp[2], done
      };
    } else {
      status.data = {
        double(wp_idx_), double(waypoints_.size()),
        0.0, 0.0, 0.0, done
      };
    }
    pub_status_->publish(status);
  }

  // ── Members ──────────────────────────────────────────────────────
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_waypoints_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_waypoint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::vector<std::array<double, 3>> waypoints_;
  std::array<double, 3> pos_{};
  bool   got_pos_{false};
  size_t wp_idx_{0};
  double arrive_dist_{DEFAULT_ARRIVE_DIST};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
