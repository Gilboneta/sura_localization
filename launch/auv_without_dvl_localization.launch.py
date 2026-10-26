import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def namespaced_frame(robot_namespace, frame_name):
    frame_name = frame_name.strip().strip("/")
    if robot_namespace:
        return f"{robot_namespace}/{frame_name}"
    return frame_name


def namespaced_topic(robot_namespace, topic_name):
    topic_name = topic_name.strip()
    if topic_name.startswith("/"):
        return topic_name
    topic_name = topic_name.strip("/")
    if robot_namespace:
        return f"/{robot_namespace}/{topic_name}"
    return f"/{topic_name}"


def load_node_parameters(config_path, node_name):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    return config.get(node_name, {}).get("ros__parameters", {})


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")

    config_package = LaunchConfiguration("config_package").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)

    config_path = os.path.join(
        get_package_share_directory(config_package),
        config_file,
    )

    ekf_params = load_node_parameters(config_path, "ekf_filter_node")

    map_frame = namespaced_frame(robot_namespace, "map")
    base_link_frame = namespaced_frame(robot_namespace, "base_link")

    aruco_share = get_package_share_directory("cirtesu_tank_aruco_localization")
    aruco_config_path = os.path.join(
        aruco_share,
        "config",
        "aruco_map.yaml",
    )
    aruco_params = load_node_parameters(
        aruco_config_path,
        "aruco_map_localization",
    )

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_cirtesu_tank",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "3.1416",
                "--frame-id", "world_ned",
                "--child-frame-id", "cirtesu_tank",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_map",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "world_ned",
                "--child-frame-id", map_frame,
            ],
        ),

        Node(
            package="cirtesu_tank_aruco_localization",
            executable="aruco_map_localization_node",
            name="aruco_map_localization",
            namespace=f"/{robot_namespace}" if robot_namespace else "",
            output="screen",
            parameters=[aruco_params],
        ),

        Node(
            package="sura_localization",
            executable="pressure_to_pose",
            name="pressure_to_pose",
            output="screen",
            parameters=[
                {
                    "input_topic": "sensors/pressure",
                    "output_topic": "sensors/pressure/pose_ned",
                    "environment": "sim",
                    "frame_id": map_frame,
                    "sensor_frame_id": namespaced_frame(
                        robot_namespace,
                        "pressure_link",
                    ),
                    "positive_down": True,
                    "z_scale": 1.0,
                    "z_offset_m": 0.01,
                    "fallback_z_variance": 0.01,
                    "fallback_xy_variance": 0.01,
                }
            ],
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_params,
                {
                    "map_frame": map_frame,
                    "odom_frame": map_frame,
                    "base_link_frame": base_link_frame,
                    "world_frame": map_frame,
                    "publish_tf": True,
                },
            ],
        ),

    ]

    return nodes

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="bluerov",
            ),
            DeclareLaunchArgument(
                "config_package",
                default_value="sura_localization",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="config/auv_without_dvl_localization.yaml",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )