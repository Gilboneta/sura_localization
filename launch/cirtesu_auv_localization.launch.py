import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def str_to_bool(value):
    return str(value).lower() in ("true", "1", "yes", "on")


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")

    world_frame = LaunchConfiguration("world_frame").perform(context).strip()
    if not world_frame:
        world_frame = LaunchConfiguration("map_frame").perform(context).strip()
    if not world_frame:
        world_frame = namespaced_frame(robot_namespace, "map")

    imu_enu_frame = LaunchConfiguration("imu_enu_frame").perform(context).strip().strip("/")
    if not imu_enu_frame:
        imu_enu_frame = namespaced_frame(robot_namespace, "imu_link_flu")

    imu_ned_topic = namespaced_topic(
        robot_namespace,
        LaunchConfiguration("imu_ned_topic").perform(context),
    )
    imu_enu_topic = namespaced_topic(
        robot_namespace,
        LaunchConfiguration("imu_enu_topic").perform(context),
    )

    imu_orientation_yaw_stddev_deg = float(
        LaunchConfiguration("imu_orientation_yaw_stddev_deg").perform(context)
    )

    aruco_share = get_package_share_directory("cirtesu_tank_aruco_localization")
    aruco_config_path = os.path.join(aruco_share, "config", "aruco_map.yaml")
    aruco_params = load_node_parameters(aruco_config_path, "aruco_map_localization")

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
            package="cirtesu_tank_aruco_localization",
            executable="aruco_map_localization_node",
            name="aruco_map_localization",
            namespace=f"/{robot_namespace}" if robot_namespace else "",
            output="screen",
            parameters=[aruco_params],
        ),
    ]

    if str_to_bool(LaunchConfiguration("convert_imu_ned_to_enu").perform(context)):
        nodes.append(
            Node(
                package="sura_localization",
                executable="imu_ned_to_enu",
                name="imu_ned_to_enu",
                output="screen",
                parameters=[
                    {
                        "input_topic": imu_ned_topic,
                        "output_topic": imu_enu_topic,
                        "output_frame": imu_enu_frame,
                        "convert_frd_to_flu": str_to_bool(
                            LaunchConfiguration("convert_frd_to_flu").perform(context)
                        ),
                        "orientation_yaw_stddev_deg": imu_orientation_yaw_stddev_deg,
                    }
                ],
            )
        )

    return nodes


def generate_launch_description():
    sura_localization_share = get_package_share_directory("sura_localization")
    auv_localization_launch = os.path.join(
        sura_localization_share,
        "launch",
        "auv_localization.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="bluerov"),
            DeclareLaunchArgument("environment", default_value="real"),
            DeclareLaunchArgument("config_package", default_value="sura_localization"),
            DeclareLaunchArgument(
                "config_file",
                default_value="config/cirtesu_auv_localization.yaml",
            ),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("odom_frame", default_value=""),
            DeclareLaunchArgument("base_link_frame", default_value=""),
            DeclareLaunchArgument("world_frame", default_value=""),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("use_navsat", default_value="true"),
            DeclareLaunchArgument("wait_for_datum", default_value="true"),
            DeclareLaunchArgument("datum_latitude"),
            DeclareLaunchArgument("datum_longitude"),
            DeclareLaunchArgument("datum_heading"),
            DeclareLaunchArgument("convert_imu_ned_to_enu", default_value="true"),
            DeclareLaunchArgument("imu_ned_topic", default_value="sensors/imu"),
            DeclareLaunchArgument("imu_enu_topic", default_value="sensors/imu_enu"),
            DeclareLaunchArgument("imu_enu_frame", default_value=""),
            DeclareLaunchArgument("convert_frd_to_flu", default_value="true"),
            DeclareLaunchArgument("imu_orientation_yaw_stddev_deg", default_value="-1.0"),
            DeclareLaunchArgument("convert_pressure_to_pose", default_value="true"),
            DeclareLaunchArgument("pressure_topic", default_value="sensors/pressure"),
            DeclareLaunchArgument("pressure_pose_topic", default_value="sensors/pressure/pose"),
            DeclareLaunchArgument("pressure_pose_z_scale", default_value="1.0"),
            DeclareLaunchArgument("pressure_pose_z_offset_m", default_value="0.01"),
            DeclareLaunchArgument("output_odom_topic", default_value=""),
            DeclareLaunchArgument("output_ned_odom_topic", default_value=""),
            DeclareLaunchArgument("ned_world_frame", default_value="world_ned"),

            OpaqueFunction(function=launch_setup),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(auv_localization_launch),
                launch_arguments=[
                    ("robot_namespace", LaunchConfiguration("robot_namespace")),
                    ("environment", LaunchConfiguration("environment")),
                    ("config_package", LaunchConfiguration("config_package")),
                    ("config_file", LaunchConfiguration("config_file")),
                    ("map_frame", LaunchConfiguration("map_frame")),
                    ("odom_frame", LaunchConfiguration("odom_frame")),
                    ("base_link_frame", LaunchConfiguration("base_link_frame")),
                    ("world_frame", LaunchConfiguration("world_frame")),
                    ("publish_tf", LaunchConfiguration("publish_tf")),
                    ("use_navsat", LaunchConfiguration("use_navsat")),
                    ("wait_for_datum", LaunchConfiguration("wait_for_datum")),
                    ("datum_latitude", LaunchConfiguration("datum_latitude")),
                    ("datum_longitude", LaunchConfiguration("datum_longitude")),
                    ("datum_heading", LaunchConfiguration("datum_heading")),
                    ("convert_imu_ned_to_enu", "false"),
                    ("imu_ned_topic", LaunchConfiguration("imu_enu_topic")),
                    ("imu_enu_topic", LaunchConfiguration("imu_enu_topic")),
                    ("imu_enu_frame", LaunchConfiguration("imu_enu_frame")),
                    (
                        "imu_orientation_yaw_stddev_deg",
                        LaunchConfiguration("imu_orientation_yaw_stddev_deg"),
                    ),
                    ("convert_pressure_to_pose", LaunchConfiguration("convert_pressure_to_pose")),
                    ("pressure_topic", LaunchConfiguration("pressure_topic")),
                    ("pressure_pose_topic", LaunchConfiguration("pressure_pose_topic")),
                    ("pressure_pose_z_scale", LaunchConfiguration("pressure_pose_z_scale")),
                    (
                        "pressure_pose_z_offset_m",
                        LaunchConfiguration("pressure_pose_z_offset_m"),
                    ),
                    ("output_odom_topic", LaunchConfiguration("output_odom_topic")),
                    ("output_ned_odom_topic", LaunchConfiguration("output_ned_odom_topic")),
                    ("ned_world_frame", LaunchConfiguration("ned_world_frame")),
                ],
            ),
        ]
    )
