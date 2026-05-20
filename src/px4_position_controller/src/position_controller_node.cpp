#include "px4_position_controller/position_controller_node.hpp"
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

static constexpr double INTEGRAL_LIM = 2.0;   // [m/s²] anti-windup
static constexpr double ACC_XY_MAX   = 5.0;   // [m/s²]
static constexpr double ACC_Z_MAX    = 5.0;   // [m/s²]

static constexpr uint64_t TICK_ARM      = 40;   // 2.0s
static constexpr uint64_t TICK_OFFBOARD = 50;   // 2.5s

namespace px4_controller
{

// ─────────────────────────────────────────────────────────────────────────────
PositionControllerNode::PositionControllerNode()
: rclcpp::Node("position_controller_node")
{
  // ── Declare parameters ─────────────────────────────────────────
  declare_parameter("active", true);
  declare_parameter("kp_xy", 0.6);
  declare_parameter("ki_xy", 0.02);
  declare_parameter("kd_xy", 0.3);
  declare_parameter("kp_z", 1.5);
  declare_parameter("ki_z", 0.10);
  declare_parameter("kd_z", 0.5);

  active_ = get_parameter("active").as_bool();
  gains_xy_ = {
    get_parameter("kp_xy").as_double(),
    get_parameter("ki_xy").as_double(),
    get_parameter("kd_xy").as_double()
  };
  gains_z_ = {
    get_parameter("kp_z").as_double(),
    get_parameter("ki_z").as_double(),
    get_parameter("kd_z").as_double()
  };

  // ── Parameter callback ─────────────────────────────────────────
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&PositionControllerNode::onParamChange, this, _1));

  rclcpp::QoS px4_qos(5);
  px4_qos.best_effort().durability_volatile();

  // ── Subscribers ────────────────────────────────────────────────
  sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position_v1", px4_qos,
    std::bind(&PositionControllerNode::onLocalPos, this, _1));

  sub_waypoint_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/waypoint/current", 10,
    std::bind(&PositionControllerNode::onWaypoint, this, _1));

  // ── Publishers ─────────────────────────────────────────────────
  pub_traj_sp_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);
  pub_offboard_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);
  pub_vehicle_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);
  pub_debug_accel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/ctrl/debug_accel", 10);
  pub_pid_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ctrl/pid_debug", 10);

  // ── Control loop 20 Hz ────────────────────────────────────────
  ctrl_timer_ = create_wall_timer(50ms,
    std::bind(&PositionControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(),
    "[PID] Khoi dong. active=%s. PID_xy=(%.2f,%.2f,%.2f) PID_z=(%.2f,%.2f,%.2f)",
    active_ ? "true" : "false",
    gains_xy_.kp, gains_xy_.ki, gains_xy_.kd,
    gains_z_.kp, gains_z_.ki, gains_z_.kd);
}

// ── Parameter change callback ────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult PositionControllerNode::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    if (p.get_name() == "active") {
      active_ = p.as_bool();
      RCLCPP_INFO(get_logger(), "[PID] active = %s", active_ ? "true" : "false");
    }
    else if (p.get_name() == "kp_xy") { gains_xy_.kp = p.as_double(); }
    else if (p.get_name() == "ki_xy") { gains_xy_.ki = p.as_double(); }
    else if (p.get_name() == "kd_xy") { gains_xy_.kd = p.as_double(); }
    else if (p.get_name() == "kp_z")  { gains_z_.kp  = p.as_double(); }
    else if (p.get_name() == "ki_z")  { gains_z_.ki  = p.as_double(); }
    else if (p.get_name() == "kd_z")  { gains_z_.kd  = p.as_double(); }
  }

  RCLCPP_INFO(get_logger(),
    "[PID] Gains updated: XY=(%.3f,%.3f,%.3f) Z=(%.3f,%.3f,%.3f)",
    gains_xy_.kp, gains_xy_.ki, gains_xy_.kd,
    gains_z_.kp, gains_z_.ki, gains_z_.kd);

  return result;
}

// ── Callback ─────────────────────────────────────────────────────────────────
void PositionControllerNode::onLocalPos(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  pos_ << msg->x, msg->y, msg->z;
  vel_ << msg->vx, msg->vy, msg->vz;
  got_state_ = true;
}
 
// ── Control loop ─────────────────────────────────────────────────────────────
void PositionControllerNode::controlLoop()
{
  if (!got_state_ || !active_) return;

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

  // Error
  Eigen::Vector3d pos_err = pos_des_ - pos_;
  Eigen::Vector3d vel_err = vel_des_ - vel_;

  // PID → acc_cmd + feedforward
  Eigen::Vector3d acc_cmd = computePID(pos_err, vel_err, dt) + acc_des_;

  // Clamp
  acc_cmd.x() = std::clamp(acc_cmd.x(), -ACC_XY_MAX, ACC_XY_MAX);
  acc_cmd.y() = std::clamp(acc_cmd.y(), -ACC_XY_MAX, ACC_XY_MAX);
  acc_cmd.z() = std::clamp(acc_cmd.z(), -ACC_Z_MAX,  ACC_Z_MAX);

  // TrajectorySetpoint: position + velocity (ff) + acceleration (controller output)
  px4_msgs::msg::TrajectorySetpoint traj{};
  traj.timestamp      = now.nanoseconds() / 1000;
  traj.position       = {float(pos_des_.x()), float(pos_des_.y()), float(pos_des_.z())};
  traj.velocity       = {float(vel_des_.x()), float(vel_des_.y()), float(vel_des_.z())};
  traj.acceleration   = {float(acc_cmd.x()),  float(acc_cmd.y()),  float(acc_cmd.z())};
  traj.yaw            = float(yaw_des_);
  pub_traj_sp_->publish(traj);
  RCLCPP_INFO(get_logger(), "[PID]: pos_err=(%.2f, %.2f, %.2f) acc_cmd=(%.2f, %.2f, %.2f)",
    pos_err.x(), pos_err.y(), pos_err.z(),
    acc_cmd.x(), acc_cmd.y(), acc_cmd.z());

  // Debug accel
  geometry_msgs::msg::Vector3Stamped dbg{};
  dbg.header.stamp = now;
  dbg.vector.x = acc_cmd.x();
  dbg.vector.y = acc_cmd.y();
  dbg.vector.z = acc_cmd.z();
  pub_debug_accel_->publish(dbg);

  // Debug: full PID state [pos_err(3), vel_err(3), integral(3), acc_cmd(3), gains(6)]
  std_msgs::msg::Float64MultiArray pid_dbg{};
  pid_dbg.data = {
    pos_err.x(), pos_err.y(), pos_err.z(),              // position error
    vel_err.x(), vel_err.y(), vel_err.z(),              // velocity error
    integral_err_.x(), integral_err_.y(), integral_err_.z(), // integral
    acc_cmd.x(), acc_cmd.y(), acc_cmd.z(),              // control output
    gains_xy_.kp, gains_xy_.ki, gains_xy_.kd,           // XY gains
    gains_z_.kp,  gains_z_.ki,  gains_z_.kd             // Z gains
  };
  pub_pid_debug_->publish(pid_dbg);
}

// ── PID ──────────────────────────────────────────────────────────────────────
Eigen::Vector3d PositionControllerNode::computePID(
  const Eigen::Vector3d & pos_err,
  const Eigen::Vector3d & vel_err,
  double dt)
{
  integral_err_ += pos_err * dt;
  for (int i = 0; i < 3; ++i) {
    integral_err_[i] = std::clamp(integral_err_[i], -INTEGRAL_LIM, INTEGRAL_LIM);
  }

  Eigen::Vector3d acc;
  acc.x() = gains_xy_.kp * pos_err.x() + gains_xy_.ki * integral_err_.x() + gains_xy_.kd * vel_err.x();
  acc.y() = gains_xy_.kp * pos_err.y() + gains_xy_.ki * integral_err_.y() + gains_xy_.kd * vel_err.y();
  acc.z() = gains_z_.kp  * pos_err.z() + gains_z_.ki  * integral_err_.z() + gains_z_.kd  * vel_err.z();
  return acc;
}

// ── PX4 helpers ──────────────────────────────────────────────────────────────
void PositionControllerNode::publishOffboardMode()
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

void PositionControllerNode::sendVehicleCommand(
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

void PositionControllerNode::arm()
{
  sendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    1.0f, 21196.0f);
  RCLCPP_INFO(get_logger(), "[PID] ARM command sent.");
}

void PositionControllerNode::setOffboardMode()
{
  sendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    1.0f, 6.0f);
  RCLCPP_INFO(get_logger(), "[PID] OFFBOARD mode command sent.");
}

// ── Waypoint callback ─────────────────────────────────────────────────────────
void PositionControllerNode::onWaypoint(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  pos_des_ << msg->point.x, msg->point.y, msg->point.z;
  integral_err_.setZero();  // reset integral khi doi waypoint
  RCLCPP_INFO(get_logger(), "[PID] Waypoint moi: (%.2f, %.2f, %.2f)",
    pos_des_.x(), pos_des_.y(), pos_des_.z());
}

}   // namespace px4_controller

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_controller::PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
