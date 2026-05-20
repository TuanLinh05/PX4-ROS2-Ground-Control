#!/usr/bin/env python3
"""
Process Manager — Launch/stop PX4 SITL, DDS Agent, and all ROS2 nodes
from a single interface.
"""

import subprocess
import threading
import time
import os
import signal
from typing import Dict, Optional


class ManagedProcess:
    """Wraps a subprocess with status tracking and log capture."""

    def __init__(self, name: str, command: str, cwd: Optional[str] = None,
                 env: Optional[dict] = None, shell: bool = True):
        self.name = name
        self.command = command
        self.cwd = cwd
        self.env = env
        self.shell = shell
        self.process: Optional[subprocess.Popen] = None
        self.status = 'stopped'  # stopped, starting, running, error
        self.log_lines = []
        self.max_log_lines = 200
        self._log_thread: Optional[threading.Thread] = None

    def start(self):
        if self.process and self.process.poll() is None:
            return  # Already running

        self.status = 'starting'
        self.log_lines = []

        try:
            env = os.environ.copy()
            if self.env:
                env.update(self.env)

            self.process = subprocess.Popen(
                self.command,
                shell=self.shell,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=self.cwd,
                env=env,
                start_new_session=True,
            )

            self.status = 'running'
            self._log_thread = threading.Thread(
                target=self._read_output, daemon=True)
            self._log_thread.start()

        except Exception as e:
            self.status = 'error'
            self.log_lines.append(f'ERROR: {e}')

    def stop(self):
        if self.process and self.process.poll() is None:
            try:
                # Send SIGTERM to process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except (ProcessLookupError, OSError):
                try:
                    self.process.terminate()
                except Exception:
                    pass

            # Wait a bit, then force kill
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except Exception:
                    self.process.kill()

        self.status = 'stopped'

    def _read_output(self):
        try:
            for line in iter(self.process.stdout.readline, b''):
                decoded = line.decode('utf-8', errors='replace').rstrip()
                self.log_lines.append(decoded)
                if len(self.log_lines) > self.max_log_lines:
                    self.log_lines.pop(0)
        except Exception:
            pass
        finally:
            if self.process and self.process.poll() is not None:
                if self.status == 'running':
                    self.status = 'stopped'

    def get_info(self) -> dict:
        # Update status if process died
        if self.process and self.process.poll() is not None and self.status == 'running':
            self.status = 'stopped'

        return {
            'name': self.name,
            'status': self.status,
            'log': self.log_lines[-20:],  # Last 20 lines for the UI
        }


class ProcessManager:
    """Manages all processes needed for the PX4 drone stack."""

    def __init__(self):
        home = os.path.expanduser('~')
        ws = os.path.join(home, 'ros2_px4_ws')

        # Source command prefix
        src = f'source /opt/ros/humble/setup.bash && source {ws}/install/setup.bash 2>/dev/null; '

        self.processes: Dict[str, ManagedProcess] = {
            'px4_sitl': ManagedProcess(
                'PX4 SITL',
                f'cd {home}/PX4-Autopilot && make px4_sitl gz_x500',
                cwd=os.path.join(home, 'PX4-Autopilot'),
            ),
            'dds_agent': ManagedProcess(
                'DDS Agent',
                'MicroXRCEAgent udp4 -p 8888',
            ),
            'pid_controller': ManagedProcess(
                'PID Controller',
                f'bash -c "{src} ros2 run px4_position_controller position_controller_node"',
            ),
            'lqr_controller': ManagedProcess(
                'LQR Controller',
                f'bash -c "{src} ros2 run px4_lqr_controller lqr_controller_node --ros-args -p active:=false"',
            ),
            'gen_trajectory': ManagedProcess(
                'Trajectory Generator',
                f'bash -c "{src} ros2 run px4_gen_trajectory gen_trajectory_node"',
            ),
            'visualizer': ManagedProcess(
                'Trajectory Visualizer',
                f'bash -c "{src} ros2 run px4_trajectory_visualizer trajectory_visualizer_node"',
            ),
            'rviz2': ManagedProcess(
                'RViz2',
                f'bash -c "{src} rviz2 -d {ws}/src/px4_trajectory_visualizer/rviz/trajectory.rviz"',
            ),
        }

    def _cleanup_zombies(self):
        """Kill any leftover PX4/Gazebo processes before launching."""
        try:
            subprocess.run('killall -9 px4 ruby gz 2>/dev/null; true',
                           shell=True, timeout=5)
            time.sleep(1)
        except Exception:
            pass

    def _configure_px4_params(self):
        """Send configuration commands to PX4 shell via stdin."""
        px4_proc = self.processes.get('px4_sitl')
        if px4_proc and px4_proc.process and px4_proc.process.poll() is None:
            try:
                if px4_proc.process.stdin:
                    cmds = [
                        'param set COM_RCL_EXCEPT 4\n',
                        'param set NAV_RCL_ACT 0\n',
                        'param set NAV_DLL_ACT 0\n',
                    ]
                    for cmd in cmds:
                        px4_proc.process.stdin.write(cmd.encode())
                        px4_proc.process.stdin.flush()
                        time.sleep(0.3)
                    print('[ProcessManager] PX4 params configured: COM_RCL_EXCEPT=4, NAV_RCL_ACT=0')
            except Exception as e:
                print(f'[ProcessManager] Failed to configure PX4 params: {e}')

    def launch_all(self):
        """Launch all processes in sequence with delays."""
        def _launch_sequence():
            # Step 0: Kill zombie processes
            self._cleanup_zombies()

            # Step 1: Launch PX4 SITL
            px4 = self.processes.get('px4_sitl')
            if px4 and px4.status != 'running':
                px4.start()

            # Wait for PX4 to be ready (look for 'pxh>' in logs, max 60s)
            print('[ProcessManager] Waiting for PX4 SITL to be ready...')
            for i in range(30):  # 30 x 2s = 60s max
                time.sleep(2)
                log_text = ' '.join(px4.log_lines[-10:]) if px4.log_lines else ''
                if 'pxh>' in log_text or 'Startup script returned' in log_text:
                    print(f'[ProcessManager] PX4 SITL ready after ~{(i+1)*2}s')
                    break
            else:
                print('[ProcessManager] WARNING: PX4 SITL may not be ready yet, continuing anyway...')

            # Configure PX4 safety parameters
            self._configure_px4_params()
            time.sleep(2)

            # Step 2: Launch remaining processes
            remaining = ['dds_agent', 'pid_controller', 'lqr_controller',
                         'gen_trajectory', 'visualizer', 'rviz2']
            delays = [5, 3, 1, 1, 1, 1]

            for name, delay in zip(remaining, delays):
                proc = self.processes.get(name)
                if proc and proc.status != 'running':
                    proc.start()
                    time.sleep(delay)

        thread = threading.Thread(target=_launch_sequence, daemon=True)
        thread.start()

    def stop_all(self):
        """Stop all processes and clean up zombie processes."""
        for proc in reversed(list(self.processes.values())):
            proc.stop()
        # Extra cleanup
        self._cleanup_zombies()

    def launch_one(self, name: str):
        if name == 'px4_sitl':
            self._cleanup_zombies()
        proc = self.processes.get(name)
        if proc:
            proc.start()

    def stop_one(self, name: str):
        proc = self.processes.get(name)
        if proc:
            proc.stop()

    def get_status(self) -> dict:
        return {name: proc.get_info() for name, proc in self.processes.items()}
