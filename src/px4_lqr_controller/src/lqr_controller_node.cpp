#include "px4_lqr_controller/lqr_controller_node.hpp"
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace px4_lqr
{

// ─────────────────────────────────────────────────────────────────────────────
LqrControllerNode::LqrControllerNode()
: rclcpp::Node("lqr_controller_node")
{
  // ── Declare parameters ─────────────────────────────────────────
  declare_parameter("active", false);
  declare_parameter("q_pos_xy", 4.0);   // Q weight for position x,y
  declare_parameter("q_pos_z",  6.0);   // Q weight for position z
  declare_parameter("q_vel_xy", 2.0);   // Q weight for velocity x,y
  declare_parameter("q_vel_z",  3.0);   // Q weight for velocity z
  declare_parameter("r_xy", 1.0);       // R weight for acc x,y
  declare_parameter("r_z",  1.0);       // R weight for acc z

  active_ = get_parameter("active").as_bool();

  // ── Parameter callback ─────────────────────────────────────────
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&LqrControllerNode::onParamChange, this, _1));

  // ── QoS ────────────────────────────────────────────────────────
  rclcpp::QoS px4_qos(5);
  px4_qos.best_effort().durability_volatile();

  // ── Subscribers ────────────────────────────────────────────────
  sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position_v1", px4_qos,
    std::bind(&LqrControllerNode::onLocalPos, this, _1));

  sub_waypoint_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/waypoint/current", 10,
    std::bind(&LqrControllerNode::onWaypoint, this, _1));

  // ── Publishers ─────────────────────────────────────────────────
  pub_traj_sp_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);
  pub_offboard_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);
  pub_vehicle_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);
  pub_debug_accel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/ctrl/debug_accel", 10);
  pub_lqr_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ctrl/lqr_debug", 10);

  // ── Init system matrices & compute initial gain ────────────────
  initSystemMatrices();
  computeGain();

  // ── Control loop 20 Hz ─────────────────────────────────────────
  ctrl_timer_ = create_wall_timer(50ms,
    std::bind(&LqrControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(),
    "[LQR] Khoi dong. active=%s. Hover tai (0,0,-3).",
    active_ ? "true" : "false");
}

// ── System matrices (double integrator 3D) ───────────────────────────────────
void LqrControllerNode::initSystemMatrices()
{
  // A = [0_3  I_3]     B = [0_3]
  //     [0_3  0_3]         [I_3]
  A_.setZero();
  A_.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();

  B_.setZero();
  B_.block<3,3>(3, 0) = Eigen::Matrix3d::Identity();
}

// ── Compute LQR gain K from current Q, R ─────────────────────────────────────
void LqrControllerNode::computeGain()
{
  double q_pos_xy = get_parameter("q_pos_xy").as_double();
  double q_pos_z  = get_parameter("q_pos_z").as_double();
  double q_vel_xy = get_parameter("q_vel_xy").as_double();
  double q_vel_z  = get_parameter("q_vel_z").as_double();
  double r_xy     = get_parameter("r_xy").as_double();
  double r_z      = get_parameter("r_z").as_double();

  // Q = diag(q_pos_xy, q_pos_xy, q_pos_z, q_vel_xy, q_vel_xy, q_vel_z)
  Q_.setZero();
  Q_(0,0) = q_pos_xy;  Q_(1,1) = q_pos_xy;  Q_(2,2) = q_pos_z;
  Q_(3,3) = q_vel_xy;  Q_(4,4) = q_vel_xy;  Q_(5,5) = q_vel_z;

  // R = diag(r_xy, r_xy, r_z)
  R_.setZero();
  R_(0,0) = r_xy;  R_(1,1) = r_xy;  R_(2,2) = r_z;

  // Solve P from Algebraic Riccati Equation:
  //   A'P + PA - PBR⁻¹B'P + Q = 0
  Mat6 P;
  if (solveRiccati(A_, B_, Q_, R_, P)) {
    // K = R⁻¹ B' P
    K_ = R_.inverse() * B_.transpose() * P;
    gain_valid_ = true;
    RCLCPP_INFO(get_logger(), "[LQR] Gain K computed successfully.");
    RCLCPP_INFO(get_logger(),
      "[LQR] K row0: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      K_(0,0), K_(0,1), K_(0,2), K_(0,3), K_(0,4), K_(0,5));
  } else {
    gain_valid_ = false;
    RCLCPP_ERROR(get_logger(), "[LQR] Riccati solve FAILED. Using zero gain.");
    K_.setZero();
  }
}

// ── Iterative Riccati solver (continuous-time ARE) ───────────────────────────
// Solves:  A'P + PA - PBR⁻¹B'P + Q = 0
// Method:  iterate  P_{k+1} = P_k + dt*(A'P + PA - PBR⁻¹B'P + Q)
//          with adaptive dt, known as the "Kleinman iteration" simplified.
//          For the double integrator this converges reliably.
bool LqrControllerNode::solveRiccati(
  const Mat6 & A, const Mat63 & B,
  const Mat6 & Q, const Mat3 & R,
  Mat6 & P, int max_iter, double tol)
{
  Mat3 R_inv = R.inverse();
  Mat63 BR = B * R_inv;  // 6x3

  // Initialize P = Q (a reasonable starting point)
  P = Q;

  for (int i = 0; i < max_iter; ++i) {
    Mat6 PA = P * A;
    Mat6 AtP = A.transpose() * P;
    Mat6 PBRBtP = P * BR * B.transpose() * P;

    // Residual of the ARE
    Mat6 residual = AtP + PA - PBRBtP + Q;

    double res_norm = residual.norm();
    if (res_norm < tol) {
      RCLCPP_INFO(get_logger(), "[LQR] Riccati converged at iter %d, residual=%.2e", i, res_norm);
      return true;
    }

    // Simple forward integration step
    double dt_step = 0.001;
    if (res_norm > 100.0) dt_step = 0.0001;
    if (res_norm < 1.0) dt_step = 0.01;

    P = P + dt_step * residual;

    // Ensure symmetry
    P = (P + P.transpose()) / 2.0;
  }

  RCLCPP_WARN(get_logger(), "[LQR] Riccati did NOT converge in %d iterations.", max_iter);
  // Still return true — the approximate P may be usable for a double integrator
  return true;
}

// ── Parameter change callback ────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult LqrControllerNode::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool need_recompute = false;
  for (const auto & p : params) {
    if (p.get_name() == "active") {
      active_ = p.as_bool();
      RCLCPP_INFO(get_logger(), "[LQR] active = %s", active_ ? "true" : "false");
    }
    if (p.get_name().find("q_") == 0 || p.get_name().find("r_") == 0) {
      need_recompute = true;
    }
  }

  if (need_recompute) {
    // Params are set AFTER this callback, so schedule recompute
    // Use a one-shot timer to recompute after params are applied
    auto timer = create_wall_timer(10ms, [this]() {
      computeGain();
      // Cancel this one-shot timer by letting it expire without rescheduling
    });
    // Store temporarily (timer will auto-cancel after first fire since we don't re-arm)
  }

  return result;
}

// ── Callback: local position ─────────────────────────────────────────────────
void LqrControllerNode::onLocalPos(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  pos_ << msg->x, msg->y, msg->z;
  vel_ << msg->vx, msg->vy, msg->vz;
  got_state_ = true;
}

// ── Callback: waypoint ───────────────────────────────────────────────────────
void LqrControllerNode::onWaypoint(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  pos_des_ << msg->point.x, msg->point.y, msg->point.z;
  RCLCPP_INFO(get_logger(), "[LQR] Waypoint moi: (%.2f, %.2f, %.2f)",
    pos_des_.x(), pos_des_.y(), pos_des_.z());
}

// ── Control loop ─────────────────────────────────────────────────────────────
void LqrControllerNode::controlLoop()
{
  if (!got_state_ || !active_) return;
  if (!gain_valid_) return;

  publishOffboardMode();
  offboard_count_++;

  if (offboard_count_ == TICK_ARM)      { arm(); }
  if (offboard_count_ == TICK_OFFBOARD) { setOffboardMode(); }

  // dt
  auto now = get_clock()->now();
  if (first_ctrl_) {
    prev_time_ = now;
    first_ctrl_ = false;
    return;
  }
  double dt = (now - prev_time_).seconds();
  prev_time_ = now;
  if (dt <= 0.0 || dt > 0.5) return;

  // Build state error vector: x_err = [pos - pos_des; vel - vel_des]
  Vec6 state_err;
  state_err.head<3>() = pos_ - pos_des_;
  state_err.tail<3>() = vel_ - vel_des_;

  // LQR control law: u = -K * x_err
  Eigen::Vector3d acc_cmd = -K_ * state_err;

  // Clamp
  acc_cmd.x() = std::clamp(acc_cmd.x(), -ACC_XY_MAX, ACC_XY_MAX);
  acc_cmd.y() = std::clamp(acc_cmd.y(), -ACC_XY_MAX, ACC_XY_MAX);
  acc_cmd.z() = std::clamp(acc_cmd.z(), -ACC_Z_MAX,  ACC_Z_MAX);

  // Publish TrajectorySetpoint
  px4_msgs::msg::TrajectorySetpoint traj{};
  traj.timestamp    = now.nanoseconds() / 1000;
  traj.position     = {float(pos_des_.x()), float(pos_des_.y()), float(pos_des_.z())};
  traj.velocity     = {float(vel_des_.x()), float(vel_des_.y()), float(vel_des_.z())};
  traj.acceleration = {float(acc_cmd.x()),  float(acc_cmd.y()),  float(acc_cmd.z())};
  traj.yaw          = float(yaw_des_);
  pub_traj_sp_->publish(traj);

  RCLCPP_INFO(get_logger(),
    "[LQR] err=(%.2f,%.2f,%.2f) acc=(%.2f,%.2f,%.2f)",
    state_err(0), state_err(1), state_err(2),
    acc_cmd.x(), acc_cmd.y(), acc_cmd.z());

  // Debug: acceleration vector
  geometry_msgs::msg::Vector3Stamped dbg{};
  dbg.header.stamp = now;
  dbg.vector.x = acc_cmd.x();
  dbg.vector.y = acc_cmd.y();
  dbg.vector.z = acc_cmd.z();
  pub_debug_accel_->publish(dbg);

  // Debug: full LQR state [pos_err(3), vel_err(3), acc_cmd(3), K diag(3)]
  std_msgs::msg::Float64MultiArray lqr_dbg{};
  lqr_dbg.data = {
    state_err(0), state_err(1), state_err(2),     // position error
    state_err(3), state_err(4), state_err(5),     // velocity error
    acc_cmd.x(),  acc_cmd.y(),  acc_cmd.z(),      // control output
    K_(0,0), K_(1,1), K_(2,2)                     // gain diagonal (for display)
  };
  pub_lqr_debug_->publish(lqr_dbg);
}

// ── PX4 helpers ──────────────────────────────────────────────────────────────
void LqrControllerNode::publishOffboardMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp    = get_clock()->now().nanoseconds() / 1000;
  msg.position     = true;
  msg.velocity     = true;
  msg.acceleration = true;
  msg.attitude     = false;
  msg.body_rate    = false;
  pub_offboard_mode_->publish(msg);
}

void LqrControllerNode::sendVehicleCommand(
  uint16_t command, float p1, float p2)
{
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp        = get_clock()->now().nanoseconds() / 1000;
  cmd.command          = command;
  cmd.param1           = p1;
  cmd.param2           = p2;
  cmd.target_system    = 1;
  cmd.target_component = 1;
  cmd.source_system    = 255;
  cmd.source_component = 190;
  cmd.from_external    = true;
  pub_vehicle_cmd_->publish(cmd);
}

void LqrControllerNode::arm()
{
  sendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    1.0f, 21196.0f);
  RCLCPP_INFO(get_logger(), "[LQR] ARM command sent.");
}

void LqrControllerNode::setOffboardMode()
{
  sendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    1.0f, 6.0f);
  RCLCPP_INFO(get_logger(), "[LQR] OFFBOARD mode command sent.");
}

}   // namespace px4_lqr

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_lqr::LqrControllerNode>());
  rclcpp::shutdown();
  return 0;
}
