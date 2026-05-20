# px4_trajectory_visualizer

Node subscribe `VehicleLocalPosition` từ PX4 và publish quỹ đạo thực tế dưới dạng `nav_msgs/Path` để visualize trong RViz2.

## Topics

| Hướng | Topic | Message Type | Ghi chú |
|---|---|---|---|
| Input | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | QoS: best_effort |
| Output | `/trajectory/path` | `nav_msgs/Path` | Frame: `map` (ENU) |

> **NED → ENU conversion:** `x_enu = y_ned`, `y_enu = x_ned`, `z_enu = -z_ned`

## Build

```bash
cd ~/ros2_px4_ws
colcon build --packages-select px4_trajectory_visualizer
source ~/ros2_px4_ws/install/setup.bash
```

## Chạy node

```bash
ros2 run px4_trajectory_visualizer trajectory_visualizer_node
```

## Test

### 1. Kiểm tra topic đang publish

```bash
ros2 topic echo /trajectory/path
```

### 2. Kiểm tra tần số publish

```bash
ros2 topic hz /trajectory/path
```

### 3. Visualize trong RViz2

```bash
rviz2
```

- **Fixed Frame:** `map`
- Add → **By topic** → `/trajectory/path` → **Path**

### 4. Giả lập input (không có PX4)

```bash
ros2 topic pub /fmu/out/vehicle_local_position_v1 px4_msgs/msg/VehicleLocalPosition \
  "{x: 1.0, y: 2.0, z: -3.0, vx: 0.0, vy: 0.0, vz: 0.0}" --rate 10
```



### 5. Test với chương trình chính V1
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
    ros2 run px4_trajectory_visualizer trajectory_visualizer_node
    ```
  #### Terminal 6
    ```bash
    rviz2
    ```
  - **Fixed Frame:** `map`
- Add → **By topic** → `/trajectory/path` → **Path**

## Tham số

| Tham số | Giá trị | Mô tả |
|---|---|---|
| `MAX_POINTS` | 2000 | Số điểm tối đa lưu trong path |