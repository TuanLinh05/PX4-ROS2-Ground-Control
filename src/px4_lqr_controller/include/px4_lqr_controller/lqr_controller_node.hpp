#ifndef PX4_LQR_CONTROLLER__LQR_CONTROLLER_NODE_HPP_
#define PX4_LQR_CONTROLLER__LQR_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace px4_lqr
{

class LqrControllerNode : public rclcpp::Node
{
public:
  LqrControllerNode();

private:
  // ── Subscribers ──────────────────────────────────────────────────
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr    sub_waypoint_;

  // ── Publishers ───────────────────────────────────────────────────
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  pub_traj_sp_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      pub_vehicle_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr  pub_debug_accel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_lqr_debug_;

  // ── Control timer (20 Hz) ────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr ctrl_timer_;

  // ── Actual state ─────────────────────────────────────────────────
  Eigen::Vector3d pos_{0, 0, 0};
  Eigen::Vector3d vel_{0, 0, 0};
  bool            got_state_{false};

  // ── Setpoint ─────────────────────────────────────────────────────
  Eigen::Vector3d pos_des_{0.0, 0.0, -3.0};
  Eigen::Vector3d vel_des_{0.0, 0.0,  0.0};
  double          yaw_des_{0.0};

  // ── LQR ──────────────────────────────────────────────────────────
  // State-space: double integrator 3D
  // x = [px, py, pz, vx, vy, vz]'  (6 states)
  // u = [ax, ay, az]'               (3 inputs)
  // A = [0_3 I_3; 0_3 0_3],  B = [0_3; I_3]
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Mat63 = Eigen::Matrix<double, 6, 3>;
  using Mat36 = Eigen::Matrix<double, 3, 6>;
  using Mat3  = Eigen::Matrix<double, 3, 3>;
  using Vec6  = Eigen::Matrix<double, 6, 1>;

  Mat6  A_;       // System matrix
  Mat63 B_;       // Input matrix
  Mat6  Q_;       // State cost (diagonal)
  Mat3  R_;       // Input cost (diagonal)
  Mat36 K_;       // LQR gain
  bool  gain_valid_{false};

  // ── Timing ───────────────────────────────────────────────────────
  rclcpp::Time prev_time_;
  bool         first_ctrl_{true};

  // ── ARM / OFFBOARD sequence ──────────────────────────────────────
  uint64_t offboard_count_{0};

  // ── Parameters ───────────────────────────────────────────────────
  bool active_{false};  // default inactive, GUI activates
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ── Acceleration limits ──────────────────────────────────────────
  static constexpr double ACC_XY_MAX = 5.0;
  static constexpr double ACC_Z_MAX  = 5.0;
  static constexpr uint64_t TICK_ARM      = 40;
  static constexpr uint64_t TICK_OFFBOARD = 50;

  // ── Callbacks ────────────────────────────────────────────────────
  void onLocalPos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onWaypoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void controlLoop();

  // ── LQR computation ──────────────────────────────────────────────
  void initSystemMatrices();
  bool solveRiccati(const Mat6 & A, const Mat63 & B,
                    const Mat6 & Q, const Mat3 & R,
                    Mat6 & P, int max_iter = 200, double tol = 1e-9);
  void computeGain();

  // ── PX4 helpers ──────────────────────────────────────────────────
  void publishOffboardMode();
  void sendVehicleCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f);
  void arm();
  void setOffboardMode();
};

}  // namespace px4_lqr
#endif  // PX4_LQR_CONTROLLER__LQR_CONTROLLER_NODE_HPP_
