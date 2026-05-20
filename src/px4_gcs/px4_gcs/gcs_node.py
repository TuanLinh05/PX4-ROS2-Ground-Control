#!/usr/bin/env python3
"""
PX4 Ground Control Station — Main Server Node
Combines: FastAPI web server + rclpy ROS2 node + WebSocket real-time bridge
"""

import os
import sys
import json
import time
import math
import asyncio
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType

from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleStatus,
    BatteryStatus,
    VehicleCommand,
    TrajectorySetpoint,
    OffboardControlMode,
)
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray, Bool

from px4_gcs.process_manager import ProcessManager
from px4_gcs.trajectory_utils import generate_trajectory

# ── Attempt to import uvicorn and fastapi ─────────────────────────────────────
try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import FileResponse
except ImportError:
    print("="*60)
    print("ERROR: FastAPI or uvicorn not installed!")
    print("Run:  pip3 install fastapi uvicorn[standard] websockets")
    print("="*60)
    sys.exit(1)


# ══════════════════════════════════════════════════════════════════════════════
#  ROS2 GCS Node — runs in a background thread
# ══════════════════════════════════════════════════════════════════════════════

class GCSRosNode(Node):
    """ROS2 node that bridges telemetry to/from the web frontend."""

    def __init__(self):
        super().__init__('gcs_node')

        # ── Shared state (thread-safe via GIL for simple reads) ───────────
        self.telemetry = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            'setpoint': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'battery': {'voltage': 0.0, 'current': 0.0, 'remaining': 0.0},
            'status': {'arming_state': 0, 'nav_state': 0, 'failsafe': False},
            'pid_debug': None,
            'lqr_debug': None,
            'trajectory_status': None,
            'timestamp': 0.0,
        }

        # ── QoS for PX4 ──────────────────────────────────────────────────
        px4_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._on_local_pos, px4_qos)

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v3',
            self._on_status, px4_qos)

        self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self._on_battery, px4_qos)

        self.create_subscription(
            Float64MultiArray,
            '/ctrl/pid_debug', self._on_pid_debug, 10)

        self.create_subscription(
            Float64MultiArray,
            '/ctrl/lqr_debug', self._on_lqr_debug, 10)

        self.create_subscription(
            Float64MultiArray,
            '/trajectory/status', self._on_traj_status, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_vehicle_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pub_waypoints = self.create_publisher(Float64MultiArray, '/trajectory/set_waypoints', 10)
        self.pub_traj_reset = self.create_publisher(Bool, '/trajectory/reset', 10)

        self.get_logger().info('[GCS] ROS2 node started.')

    # ── Telemetry callbacks ───────────────────────────────────────────────
    def _on_local_pos(self, msg: VehicleLocalPosition):
        self.telemetry['position'] = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        self.telemetry['velocity'] = {'vx': msg.vx, 'vy': msg.vy, 'vz': msg.vz}
        self.telemetry['timestamp'] = time.time()

    def _on_status(self, msg: VehicleStatus):
        self.telemetry['status'] = {
            'arming_state': int(msg.arming_state),
            'nav_state': int(msg.nav_state),
            'failsafe': bool(msg.failsafe),
        }

    def _on_battery(self, msg: BatteryStatus):
        self.telemetry['battery'] = {
            'voltage': float(msg.voltage_v),
            'current': float(msg.current_a),
            'remaining': float(msg.remaining) * 100.0,
        }

    def _on_pid_debug(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 18:
            self.telemetry['pid_debug'] = {
                'pos_err': [d[0], d[1], d[2]],
                'vel_err': [d[3], d[4], d[5]],
                'integral': [d[6], d[7], d[8]],
                'acc_cmd': [d[9], d[10], d[11]],
                'gains_xy': [d[12], d[13], d[14]],
                'gains_z': [d[15], d[16], d[17]],
            }

    def _on_lqr_debug(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 12:
            self.telemetry['lqr_debug'] = {
                'pos_err': [d[0], d[1], d[2]],
                'vel_err': [d[3], d[4], d[5]],
                'acc_cmd': [d[6], d[7], d[8]],
                'gain_diag': [d[9], d[10], d[11]],
            }

    def _on_traj_status(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 6:
            self.telemetry['trajectory_status'] = {
                'current_wp': int(d[0]),
                'total_wp': int(d[1]),
                'wp_pos': [d[2], d[3], d[4]],
                'done': bool(d[5] > 0.5),
            }

    # ── Command methods (called from web server thread) ───────────────────
    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 255
        cmd.source_component = 190
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_cmd.publish(cmd)

    def arm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        self.get_logger().info('[GCS] ARM sent')

    def disarm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        self.get_logger().info('[GCS] DISARM sent')

    def set_offboard(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('[GCS] OFFBOARD mode sent')

    def land(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('[GCS] LAND sent')

    def return_to_launch(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info('[GCS] RTL sent')

    def send_waypoints(self, waypoints_flat):
        """Send waypoints as flat array [x1,y1,z1, x2,y2,z2, ...]"""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in waypoints_flat]
        self.pub_waypoints.publish(msg)
        self.get_logger().info(f'[GCS] Sent {len(waypoints_flat)//3} waypoints')

    def reset_trajectory(self):
        msg = Bool()
        msg.data = True
        self.pub_traj_reset.publish(msg)
        self.get_logger().info('[GCS] Trajectory reset')

    def set_controller_param(self, node_name, param_name, value):
        """Set a parameter on a remote ROS2 node."""
        cli = self.create_client(
            SetParameters,
            f'/{node_name}/set_parameters'
        )
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service /{node_name}/set_parameters not available')
            return False

        req = SetParameters.Request()
        param = RclParameter()
        param.name = param_name

        pval = ParameterValue()
        if isinstance(value, bool):
            pval.type = ParameterType.PARAMETER_BOOL
            pval.bool_value = value
        elif isinstance(value, float):
            pval.type = ParameterType.PARAMETER_DOUBLE
            pval.double_value = value
        elif isinstance(value, int):
            pval.type = ParameterType.PARAMETER_INTEGER
            pval.integer_value = value

        param.value = pval
        req.parameters = [param]

        future = cli.call_async(req)
        # We don't wait for result to avoid blocking
        self.get_logger().info(f'[GCS] Set {node_name}/{param_name} = {value}')
        return True


# ══════════════════════════════════════════════════════════════════════════════
#  FastAPI Web Server
# ══════════════════════════════════════════════════════════════════════════════

def create_app(ros_node: GCSRosNode, proc_manager: ProcessManager) -> FastAPI:
    app = FastAPI(title="PX4 GCS")

    # ── Resolve static files path ─────────────────────────────────────────
    # When installed via colcon, static files are in share/px4_gcs/static/
    pkg_share = None
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('px4_gcs')
        static_dir = os.path.join(pkg_share, 'static')
    except Exception:
        # Fallback: look relative to this file
        static_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')

    if not os.path.isdir(static_dir):
        # Another fallback: workspace source directory
        static_dir = os.path.expanduser('~/ros2_px4_ws/src/px4_gcs/static')

    print(f"[GCS] Serving static files from: {static_dir}")

    # ── Routes ────────────────────────────────────────────────────────────
    @app.get("/")
    async def index():
        return FileResponse(os.path.join(static_dir, 'index.html'))

    # Mount static files
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

    # ── WebSocket endpoint ────────────────────────────────────────────────
    connected_clients = set()

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        await ws.accept()
        connected_clients.add(ws)
        print(f"[GCS] WebSocket client connected. Total: {len(connected_clients)}")

        # Start telemetry streaming task
        telemetry_task = asyncio.create_task(stream_telemetry(ws))

        try:
            while True:
                data = await ws.receive_text()
                try:
                    msg = json.loads(data)
                    handle_ws_message(msg, ros_node, proc_manager)
                except json.JSONDecodeError:
                    pass
        except WebSocketDisconnect:
            pass
        except Exception as e:
            print(f"[GCS] WebSocket error: {e}")
        finally:
            connected_clients.discard(ws)
            telemetry_task.cancel()
            print(f"[GCS] WebSocket client disconnected. Total: {len(connected_clients)}")

    async def stream_telemetry(ws: WebSocket):
        """Send telemetry data to client at ~20 Hz."""
        try:
            while True:
                payload = {
                    'type': 'telemetry',
                    'data': ros_node.telemetry,
                    'processes': proc_manager.get_status(),
                    'server_time': time.time(),
                }
                # Clean NaN/Inf for JSON serialization
                payload_str = json.dumps(payload, default=_json_default)
                await ws.send_text(payload_str)
                await asyncio.sleep(0.05)  # 20 Hz
        except asyncio.CancelledError:
            pass
        except Exception:
            pass

    return app


def _json_default(obj):
    """Handle non-serializable types."""
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return 0.0
    return str(obj)


def handle_ws_message(msg: dict, ros_node: GCSRosNode, proc_manager: ProcessManager):
    """Handle incoming WebSocket commands from the frontend."""
    cmd = msg.get('cmd')

    if cmd == 'arm':
        ros_node.arm()
    elif cmd == 'disarm':
        ros_node.disarm()
    elif cmd == 'offboard':
        ros_node.set_offboard()
    elif cmd == 'land':
        ros_node.land()
    elif cmd == 'rtl':
        ros_node.return_to_launch()

    elif cmd == 'set_waypoints':
        waypoints = msg.get('waypoints', [])
        ros_node.send_waypoints(waypoints)

    elif cmd == 'reset_trajectory':
        ros_node.reset_trajectory()

    elif cmd == 'switch_controller':
        mode = msg.get('mode', 'pid')
        if mode == 'pid':
            ros_node.set_controller_param('position_controller_node', 'active', True)
            ros_node.set_controller_param('lqr_controller_node', 'active', False)
        else:
            ros_node.set_controller_param('position_controller_node', 'active', False)
            ros_node.set_controller_param('lqr_controller_node', 'active', True)

    elif cmd == 'set_pid_params':
        params = msg.get('params', {})
        for key, val in params.items():
            ros_node.set_controller_param('position_controller_node', key, float(val))

    elif cmd == 'set_lqr_params':
        params = msg.get('params', {})
        for key, val in params.items():
            ros_node.set_controller_param('lqr_controller_node', key, float(val))

    elif cmd == 'launch_all':
        proc_manager.launch_all()
    elif cmd == 'stop_all':
        proc_manager.stop_all()
    elif cmd == 'launch_process':
        name = msg.get('name', '')
        proc_manager.launch_one(name)
    elif cmd == 'stop_process':
        name = msg.get('name', '')
        proc_manager.stop_one(name)

    elif cmd == 'generate_trajectory':
        shape = msg.get('shape', 'square')
        params = msg.get('params', {})
        waypoints = generate_trajectory(shape, params)
        # Auto-send to trajectory node
        flat = []
        for wp in waypoints:
            flat.extend(wp)
        ros_node.send_waypoints(flat)


# ══════════════════════════════════════════════════════════════════════════════
#  Main entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    ros_node = GCSRosNode()
    proc_manager = ProcessManager()

    # Run rclpy spin in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Create and run FastAPI app
    app = create_app(ros_node, proc_manager)

    print("="*60)
    print("  PX4 Ground Control Station")
    print("  Open browser: http://localhost:8085")
    print("="*60)

    try:
        uvicorn.run(app, host="0.0.0.0", port=8085, log_level="warning")
    except KeyboardInterrupt:
        pass
    finally:
        proc_manager.stop_all()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
