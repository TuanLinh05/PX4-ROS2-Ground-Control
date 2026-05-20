#ifndef PX4_TEST_BRIDGE__PX4_BRIDGE_NODE_HPP_
#define PX4_TEST_BRIDGE__PX4_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

namespace px4_test_bridge
{
class Px4BridgeNode : public rclcpp::Node
{
public:
  Px4BridgeNode();

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr        sub_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr        sub_battery_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr          pub_cmd_;
  rclcpp::TimerBase::SharedPtr arm_timer_;

  bool got_status_{false};
  bool arm_sent_{false};

  void onStatus  (const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onLocalPos(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onBattery (const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void sendArmCommand(bool arm);
};
}  // namespace px4_test_bridge
#endif
