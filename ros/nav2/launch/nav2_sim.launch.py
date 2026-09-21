from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    bridge_file = LaunchConfiguration("bridge_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value="/opt/amr-nav2/ros/nav2/config/nav2_sim.yaml",
            ),
            DeclareLaunchArgument(
                "map", default_value="/opt/amr-nav2/maps/amr_world.yaml"
            ),
            DeclareLaunchArgument(
                "bridge_file",
                default_value="/opt/amr-nav2/ros/nav2/config/bridge.yaml",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="amr_gz_bridge",
                output="screen",
                parameters=[{"config_file": bridge_file}],
            ),
            Node(
                package="nav2_gateway",
                executable="simulator_tf",
                name="amr_simulator_tf",
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"yaml_filename": map_file, "use_sim_time": True},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "use_sim_time": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("cmd_vel", "cmd_vel"),
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("cmd_vel", "cmd_vel"),
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[params_file],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "use_sim_time": True,
                        "node_names": [
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                        ],
                    }
                ],
            ),
            Node(
                package="nav2_gateway",
                executable="gateway",
                name="amr_nav2_gateway",
                output="screen",
                parameters=[{"http_host": "0.0.0.0", "http_port": 8010}],
            ),
        ]
    )
