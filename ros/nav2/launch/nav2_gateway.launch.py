from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nav2_gateway",
                executable="simulator_tf",
                name="amr_simulator_tf",
                output="screen",
            ),
            Node(
                package="nav2_gateway",
                executable="gateway",
                name="amr_nav2_gateway",
                output="screen",
                parameters=[
                    {"http_host": "0.0.0.0", "http_port": 8010},
                ],
            )
        ]
    )
