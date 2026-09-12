from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_config = PathJoinSubstitution(
        [FindPackageShare("amr_perception"), "config", "perception.yaml"]
    )
    mcu_config = PathJoinSubstitution(
        [FindPackageShare("amr_mcu_bridge"), "config", "mcu.yaml"]
    )
    audio_config = PathJoinSubstitution(
        [FindPackageShare("amr_audio"), "config", "audio.yaml"]
    )
    return LaunchDescription(
        [
            LogInfo(
                msg="Hardware mode starts perception, MCU, safety and audio nodes; external OAK-D, STL-27L and cliff drivers are required. Motor enable remains false until commissioning."
            ),
            ExecuteProcess(
                cmd=["python3", "-m", "monitoring.server", "--host", "127.0.0.1"],
                output="screen",
            ),
            Node(
                package="amr_perception",
                executable="range_fusion_node",
                parameters=[perception_config],
                output="screen",
            ),
            Node(
                package="amr_perception",
                executable="camera_path_node",
                parameters=[perception_config],
                output="screen",
            ),
            Node(
                package="amr_perception",
                executable="path_guidance_node",
                parameters=[perception_config],
                output="screen",
            ),
            Node(
                package="amr_safety_node",
                executable="safety_controller_node",
                parameters=[{"drive_enable": False}],
                output="screen",
            ),
            Node(
                package="amr_mcu_bridge",
                executable="mcu_bridge_node",
                parameters=[mcu_config],
                output="screen",
            ),
            Node(
                package="amr_audio",
                executable="audio_node",
                parameters=[audio_config],
                output="screen",
            ),
            Node(
                package="amr_monitoring_adapter",
                executable="monitoring_adapter_node",
                output="screen",
            ),
        ]
    )
