//
// test_commands.cpp
// Mục đích: kiểm tra pipeline gửi lệnh ARM + OFFBOARD tới PX4
// và đọc VehicleCommandAck để xác nhận PX4 có nhận và chấp nhận không.
//
// Chạy: ros2 run px4_position_controller test_commands
//

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;

class TestCommandsNode : public rclcpp::Node
{
public:
  TestCommandsNode() : rclcpp::Node("test_commands_node")
  {
    rclcpp::QoS px4_qos(5);
    px4_qos.best_effort().durability_volatile();

    // Subscribe ACK từ PX4
    sub_ack_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack_v1", px4_qos,
      [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
        const char * result_str = "UNKNOWN";
        switch (msg->result) {
          case 0: result_str = "ACCEPTED";            break;
          case 1: result_str = "TEMPORARILY_REJECTED"; break;
          case 2: result_str = "DENIED";              break;
          case 3: result_str = "UNSUPPORTED";         break;
          case 4: result_str = "FAILED";              break;
          case 5: result_str = "IN_PROGRESS";         break;
        }
        RCLCPP_INFO(get_logger(),
          "[ACK] command=%u  result=%s (%u)",
          msg->command, result_str, msg->result);
      });

    pub_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);
    pub_offboard_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);

    // Timer 20 Hz — offboard heartbeat + sequence ARM/OFFBOARD
    timer_ = create_wall_timer(50ms, std::bind(&TestCommandsNode::tick, this));

    RCLCPP_INFO(get_logger(), "[TEST] Node started. Sequence: ARM @ 2s, OFFBOARD @ 2.5s");
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr sub_ack_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr         pub_cmd_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr    pub_offboard_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t tick_count_{0};

  void tick()
  {
    publishOffboardHeartbeat();
    tick_count_++;

    if (tick_count_ == 40) { sendArm(); }
    if (tick_count_ == 50) { sendOffboardMode(); }
  }

  void publishOffboardHeartbeat()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp  = now().nanoseconds() / 1000;
    msg.attitude   = true;
    msg.position   = false;
    msg.velocity   = false;
    msg.acceleration = false;
    msg.body_rate  = false;
    pub_offboard_->publish(msg);
  }

  void sendVehicleCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp        = now().nanoseconds() / 1000;
    cmd.command          = command;
    cmd.param1           = p1;
    cmd.param2           = p2;
    cmd.target_system    = 1;
    cmd.target_component = 1;
    cmd.source_system    = 1;
    cmd.source_component = 1;
    cmd.from_external    = true;
    pub_cmd_->publish(cmd);
    RCLCPP_INFO(get_logger(), "[SEND] command=%u  p1=%.1f  p2=%.1f", command, p1, p2);
  }

  void sendArm()
  {
    RCLCPP_INFO(get_logger(), "[TEST] >>> Sending ARM <<<");
    sendVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0f, 21196.0f);
  }

  void sendOffboardMode()
  {
    RCLCPP_INFO(get_logger(), "[TEST] >>> Sending OFFBOARD mode <<<");
    sendVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0f, 6.0f);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestCommandsNode>());
  rclcpp::shutdown();
  return 0;
}
