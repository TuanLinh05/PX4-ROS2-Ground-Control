from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# def generate_launch_description():
#     # ====== Tham số kết nối Jetson Nano ↔ Pixhawk FMUv2 ======
#     # Nếu nối qua TELEM2 + FTDI USB → thường là /dev/ttyUSB0
#     # Nếu nối thẳng UART chân J41 của Nano → /dev/ttyTHS1
#     serial_dev = DeclareLaunchArgument('serial_dev', default_value='/dev/ttyUSB0')
#     baudrate   = DeclareLaunchArgument('baudrate',   default_value='921600')

#     # ---------- micro-ROS Agent (process có sẵn, không phải node tự code) ----------
#     micro_ros_agent = ExecuteProcess(
#         cmd=[
#             'MicroXRCEAgent', 'serial',
#             '--dev',  LaunchConfiguration('serial_dev'),
#             '-b',     LaunchConfiguration('baudrate'),
#         ],
#         output='screen'
#     )

#     # ---------- Node test của chúng ta ----------
#     # Delay 3s để agent kịp thiết lập session với PX4 trước
#     bridge_node = TimerAction(
#         period=3.0,
#         actions=[
#             Node(
#                 package='px4_test_bridge',
#                 executable='px4_bridge_node',
#                 name='px4_bridge_node',
#                 output='screen',
#             )
#         ]
#     )

#     return LaunchDescription([serial_dev, baudrate, micro_ros_agent, bridge_node])
def generate_launch_description():
    udp_port = DeclareLaunchArgument(
        'udp_port', default_value='8888',
        description='UDP port mà MicroXRCEAgent lắng nghe (phải khớp với XRCE_DDS_UDP_PORT trên PX4 SITL)'
    )

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'udp4',
            '-p', LaunchConfiguration('udp_port'),
        ],
        output='screen'
    )

    bridge_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='px4_test_bridge',
                executable='px4_bridge_node',
                name='px4_bridge_node',
                output='screen',
            )
        ]
    )

    return LaunchDescription([udp_port, micro_ros_agent, bridge_node])