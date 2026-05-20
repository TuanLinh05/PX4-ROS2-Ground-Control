#include "px4_test_bridge/px4_bridge_node.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace px4_test_bridge
{

Px4BridgeNode::Px4BridgeNode() : rclcpp::Node("px4_bridge_node")
{
  // QoS phải match PX4: BEST_EFFORT + KEEP_LAST(5) + VOLATILE
  rclcpp::QoS px4_qos(5);
  px4_qos.best_effort();
  px4_qos.durability_volatile();

  sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v3", px4_qos,
    std::bind(&Px4BridgeNode::onStatus, this, _1));

  sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position_v1", px4_qos,
    std::bind(&Px4BridgeNode::onLocalPos, this, _1));

  sub_battery_ = create_subscription<px4_msgs::msg::BatteryStatus>(
    "/fmu/out/battery_status_v1", px4_qos,
    std::bind(&Px4BridgeNode::onBattery, this, _1));

  pub_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);

  arm_timer_ = create_wall_timer(5s, [this]() {
    if (!got_status_) {
      RCLCPP_WARN(get_logger(),
        "Chua nhan duoc vehicle_status — kiem tra agent / UART.");
      return;
    }
    if (!arm_sent_) {
      RCLCPP_INFO(get_logger(), ">>> Gui lenh ARM toi PX4 <<<");
      sendArmCommand(true);
      arm_sent_ = true;
    }
  });

  RCLCPP_INFO(get_logger(), "px4_bridge_node da khoi dong.");
}

void Px4BridgeNode::onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  got_status_ = true;
  RCLCPP_INFO(get_logger(),
    "[STATUS] arming_state=%u nav_state=%u failsafe=%d",
    msg->arming_state, msg->nav_state, msg->failsafe);
}

void Px4BridgeNode::onLocalPos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(),
    "[LOCAL_POS] xyz=(%.2f, %.2f, %.2f) vxyz=(%.2f, %.2f, %.2f)",
    msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz);
}

void Px4BridgeNode::onBattery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(),
    "[BATTERY] V=%.2fV I=%.2fA remaining=%.0f%%",
    msg->voltage_v, msg->current_a, msg->remaining * 100.0f);
}

void Px4BridgeNode::sendArmCommand(bool arm)
{
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.command          = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  cmd.param1           = arm ? 1.0f : 0.0f;
  cmd.param2           = 21196.0f;  // FORCE — chỉ dùng khi đã tháo cánh quạt
  cmd.target_system    = 1;
  cmd.target_component = 1;
  cmd.source_system    = 1;
  cmd.source_component = 1;
  cmd.from_external    = true;
  cmd.timestamp        = get_clock()->now().nanoseconds() / 1000;
  pub_cmd_->publish(cmd);
}

}  // namespace px4_test_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_test_bridge::Px4BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
