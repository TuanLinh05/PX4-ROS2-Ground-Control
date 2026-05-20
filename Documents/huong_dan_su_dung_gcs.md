# Hướng Dẫn Sử Dụng PX4 Ground Control Station (GCS)

## 1. Yêu Cầu
- Ubuntu 22.04 (WSL hoặc native)
- ROS2 Humble đã cài đặt
- PX4 Autopilot đã clone tại `~/PX4-Autopilot`
- MicroXRCEAgent đã build
- Python packages: `pip3 install fastapi uvicorn[standard] websockets`

## 2. Build
```bash
cd ~/ros2_px4_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 3. Chạy GCS
Chỉ cần **1 lệnh duy nhất**:
```bash
cd ~/ros2_px4_ws
source install/setup.bash
ros2 run px4_gcs gcs_node
```

Sau đó mở browser trên Windows tại: **http://localhost:8085**

## 4. Sử Dụng Giao Diện

### 4.1. Launch All (thay thế 7 terminal)
- Vào tab **📡 Processes** ở sidebar bên trái
- Nhấn **🚀 Launch All** → hệ thống sẽ tự động khởi chạy:
  1. PX4 SITL + Gazebo
  2. DDS Agent
  3. PID Controller
  4. LQR Controller
  5. Trajectory Generator
  6. Trajectory Visualizer
- Mỗi process sẽ hiển thị trạng thái 🔴 → 🟡 → 🟢

### 4.2. Thiết Kế Quỹ Đạo
- Chọn hình dạng: Square, Circle, Triangle, Hexagon, Octagon, Figure-8, Star, Helix
- Hoặc chọn **✎ Free Draw** để vẽ tự do:
  - Click: thêm waypoint
  - Kéo: di chuyển waypoint
  - Double-click: xóa waypoint
- Điều chỉnh: số lượng waypoints, kích thước, altitude, vị trí tâm
- Nhấn **🚀 Upload to Drone** để gửi xuống drone

### 4.3. Chuyển Đổi PID ↔ LQR
- Vào tab **🎮 Commands**
- Dùng toggle switch **PID ↔ LQR** để chuyển bộ điều khiển
- Giao diện tự động chuyển panel tuning tương ứng

### 4.4. Tuning Tham Số
- Vào tab **🔧 Tuning**
- **PID**: Điều chỉnh Kp, Ki, Kd cho trục XY và Z riêng biệt
- **LQR**: Điều chỉnh Q matrix (state cost) và R matrix (input cost)
- Nhấn **Apply** để gửi tham số → controller tự cập nhật real-time

### 4.5. Xem Kết Quả Live
- 4 biểu đồ Plotly cập nhật real-time:
  1. **Position vs Setpoint**: So sánh vị trí thực vs mong muốn
  2. **Error**: Sai số vị trí theo thời gian
  3. **Controller Output**: Gia tốc điều khiển
  4. **2D Trajectory**: Quỹ đạo XY thực vs kế hoạch

### 4.6. Điều Khiển Bay
- Tab **🎮 Commands**: ARM, DISARM, OFFBOARD, LAND, RTL
- ARM có hộp thoại xác nhận để tránh nhầm

### 4.7. Xuất Dữ Liệu
- Tab **📊 Data** → nhấn **📁 Export CSV** để lưu toàn bộ telemetry

## 5. Các Lưu Ý
- GCS server chạy ở port 8080, nếu trên WSL có thể truy cập trực tiếp từ Windows browser
- Khi chuyển controller, hệ thống tự động deactivate bộ cũ và activate bộ mới
- LQR controller sẽ tự tính lại gain K khi bạn thay đổi Q hoặc R

---
*Tạo bởi GCS project — ros2_px4_ws*
