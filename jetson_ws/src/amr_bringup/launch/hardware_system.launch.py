from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg="Hardware mode expects external LiDAR, cliff and STM32 bridge nodes; motor enable remains external."
            ),
            ExecuteProcess(
                cmd=["python3", "-m", "monitoring.server", "--host", "127.0.0.1"],
                output="screen",
            ),
            Node(
                package="amr_safety_node",
                executable="safety_controller_node",
                parameters=[{"drive_enable": False}],
                output="screen",
            ),
            Node(
                package="amr_monitoring_adapter",
                executable="monitoring_adapter_node",
                output="screen",
            ),
        ]
    )
