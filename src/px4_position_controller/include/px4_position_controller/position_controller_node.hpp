#ifndef PX4_POSITION_CONTROLLER__POSITION_CONTROLLER_NODE_HPP_
#define PX4_POSITION_CONTROLLER__POSITION_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Core>

namespace px4_controller
{

struct PidGains {
  double kp{0.0}, ki{0.0}, kd{0.0};
};

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode();

private:
  // ── Subscribers ──────────────────────────────────────────────────
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr  sub_local_pos_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr    sub_waypoint_;

  // ── Publishers ───────────────────────────────────────────────────
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  pub_traj_sp_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      pub_vehicle_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_debug_accel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_pid_debug_;

  // ── Control timer (20 Hz) ────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr ctrl_timer_;

  // ── Actual state ─────────────────────────────────────────────────
  Eigen::Vector3d pos_{0, 0, 0};   // NED [m]
  Eigen::Vector3d vel_{0, 0, 0};   // NED [m/s]
  bool            got_state_{false};

  // ── Setpoint (hardcoded test case) ───────────────────────────────
  // Phase 1: Hover tại z=-3m
  // Phase 2 (sau 10s): Step sang x=3m
  Eigen::Vector3d pos_des_{0.0, 0.0, -3.0};  // NED [m]
  Eigen::Vector3d vel_des_{0.0, 0.0,  0.0};  // NED [m/s]  - feedforward
  Eigen::Vector3d acc_des_{0.0, 0.0,  0.0};  // NED [m/s²] - feedforward
  double          yaw_des_{0.0};              // [rad]

  // ── PID ──────────────────────────────────────────────────────────
  PidGains        gains_xy_;
  PidGains        gains_z_;
  Eigen::Vector3d integral_err_{0, 0, 0};
  rclcpp::Time    prev_time_;
  bool            first_ctrl_{true};

  // ── ARM / OFFBOARD sequence ──────────────────────────────────────
  uint64_t offboard_count_{0};

  // ── Active parameter (for controller switching) ──────────────────
  bool active_{true};
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ── Callbacks ────────────────────────────────────────────────────
  void onLocalPos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onWaypoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void controlLoop();

  // ── Controller ───────────────────────────────────────────────────
  Eigen::Vector3d computePID(const Eigen::Vector3d & pos_err,
                              const Eigen::Vector3d & vel_err,
                              double dt);

  // ── PX4 helpers ──────────────────────────────────────────────────
  void publishOffboardMode();
  void sendVehicleCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f);
  void arm();
  void setOffboardMode();
};

}  // namespace px4_controller
#endif  // PX4_POSITION_CONTROLLER__POSITION_CONTROLLER_NODE_HPP_
