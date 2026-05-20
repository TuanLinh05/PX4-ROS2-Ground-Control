Inputs (Subscribers):

  ┌────────────────────────────────────────────┬───────────────────────────────┐
  │                   Topic                    │         Message Type          │
  ├────────────────────────────────────────────┼───────────────────────────────┤
  │ /fmu/out/vehicle_local_position (mặc định) │ px4_msgs/VehicleLocalPosition │
  └────────────────────────────────────────────┴───────────────────────────────┘

  Outputs (Publishers):

  ┌─────────────────────┬──────────────────────────────┬─────────────────────────────────────┐
  │        Topic        │         Message Type         │              Mục đích               │
  ├─────────────────────┼──────────────────────────────┼─────────────────────────────────────┤
  │ TrajectorySetpoint  │ px4_msgs/TrajectorySetpoint  │ Gửi setpoint vị trí/vận tốc cho PX4 │
  ├─────────────────────┼──────────────────────────────┼─────────────────────────────────────┤
  │ OffboardControlMode │ px4_msgs/OffboardControlMode │ Kích hoạt offboard mode             │
  ├─────────────────────┼──────────────────────────────┼─────────────────────────────────────┤
  │ VehicleCommand      │ px4_msgs/VehicleCommand      │ Gửi lệnh arm/disarm/mode            │
  ├─────────────────────┼──────────────────────────────┼─────────────────────────────────────┤
  │ debug_accel         │ geometry_msgs/Vector3Stamped │ Debug gia tốc                       │
  └─────────────────────┴──────────────────────────────┴─────────────────────────────────────┘

  Node nhận vị trí thực tế của drone, tính toán và publish trajectory setpoint để điều khiển vị trí qua
  offboard mode.