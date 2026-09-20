from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    bridge_file = LaunchConfiguration("bridge_file")
    nav2_bringup = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value="/opt/amr-nav2/config/nav2_sim.yaml"
            ),
            DeclareLaunchArgument(
                "map", default_value="/opt/amr-nav2/maps/amr_world.yaml"
            ),
            DeclareLaunchArgument(
                "bridge_file", default_value="/opt/amr-nav2/config/bridge.yaml"
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup),
                launch_arguments={
                    "map": map_file,
                    "params_file": params_file,
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_localization": "False",
                }.items(),
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
