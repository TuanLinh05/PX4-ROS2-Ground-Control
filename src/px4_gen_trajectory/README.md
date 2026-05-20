# px4_gen_trajectory

Node sinh quỹ đạo hình vuông dưới dạng danh sách waypoint và publish từng điểm đến `position_controller_node`. Chuyển waypoint tiếp theo khi drone đến đủ gần điểm hiện tại.

## Topics

| Hướng | Topic | Message Type | Ghi chú |
|---|---|---|---|
| Input | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | QoS: best_effort |
| Output | `/waypoint/current` | `geometry_msgs/PointStamped` | Frame: `map` (NED) |

## Waypoints hình vuông (NED)

| Index | x (North) | y (East) | z (Down) | Mô tả |
|---|---|---|---|---|
| WP0 | 0.0 | 0.0 | -3.0 | Hover |
| WP1 | 3.0 | 0.0 | -3.0 | North 3m |
| WP2 | 3.0 | 3.0 | -3.0 | North+East |
| WP3 | 0.0 | 3.0 | -3.0 | East 3m |
| WP4 | 0.0 | 0.0 | -3.0 | Về origin |

## Build

```bash
cd ~/ros2_px4_ws
colcon build --packages-select px4_gen_trajectory
source ~/ros2_px4_ws/install/setup.bash
```

## Chạy node

```bash
ros2 run px4_gen_trajectory gen_trajectory_node
```

## Test

### 1. Kiểm tra waypoint đang publish

```bash
ros2 topic echo /waypoint/current
```

### 2. Kiểm tra tần số publish

```bash
ros2 topic hz /waypoint/current
```

### 3. Giả lập input (không có PX4)

```bash
ros2 topic pub /fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition \
  "{x: 0.0, y: 0.0, z: -3.0, vx: 0.0, vy: 0.0, vz: 0.0}" --rate 10
```

Node sẽ nhận vị trí `(0,0,-3)` → đủ gần WP0 → tự động chuyển sang WP1.

### 4. Test với chương trình chính V1

  #### Terminal 1
    ```bash
    cd ~/PX4-Autopilot/ && make px4_sitl gz_x500
    ```
  #### Terminal 2
    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```
  #### Terminal 3
    ```bash
    cd ~
    ./QGroundControl-x86_64.AppImage
    ```
  #### Terminal 4
    ```bash
    source ~/ros2_px4_ws/install/setup.bash
    ros2 run px4_position_controller position_controller_node
    ```
  #### Terminal 5
    ```bash
    source ~/ros2_px4_ws/install/setup.bash
    ros2 run px4_gen_trajectory gen_trajectory_node
    ```
  #### Terminal 6 (optional)
    ```bash
    source ~/ros2_px4_ws/install/setup.bash
    ros2 run px4_trajectory_visualizer trajectory_visualizer_node
    ```
  #### Terminal 7 (optional)
    ```bash
    rviz2
    ```
  - **Fixed Frame:** `map`
  - Add → **By topic** → `/trajectory/path` → **Path**

## Tham số

| Tham số | Giá trị | Mô tả |
|---|---|---|
| `ARRIVE_DIST` | 0.4 m | Ngưỡng khoảng cách để chuyển waypoint tiếp theo |
