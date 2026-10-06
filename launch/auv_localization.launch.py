import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bool_launch_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False

    raise RuntimeError(
        f"Unsupported value '{value}' for launch argument '{name}'. Use true or false."
    )


def namespaced_frame(robot_namespace, frame_name):
    if robot_namespace:
        return f"{robot_namespace}/{frame_name}"
    return frame_name


def load_node_parameters(config_path, node_name):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    return config.get(node_name, {}).get("ros__parameters", {})


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    config_package = LaunchConfiguration("config_package").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    config_path = os.path.join(get_package_share_directory(config_package), config_file)
    ekf_params = load_node_parameters(config_path, "ekf_filter_node")
    navsat_params = load_node_parameters(config_path, "navsat_transform_node")

    map_frame = LaunchConfiguration("map_frame").perform(context)
    if not map_frame:
        map_frame = namespaced_frame(robot_namespace, "map")

    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    if not odom_frame:
        odom_frame = map_frame

    world_frame = LaunchConfiguration("world_frame").perform(context)
    if not world_frame:
        world_frame = map_frame

    base_link_frame = LaunchConfiguration("base_link_frame").perform(context)
    if not base_link_frame:
        base_link_frame = namespaced_frame(robot_namespace, "base_link")

    imu_enu_frame = LaunchConfiguration("imu_enu_frame").perform(context)
    if not imu_enu_frame:
        imu_enu_frame = namespaced_frame(robot_namespace, "imu_link")

    output_odom_topic = LaunchConfiguration("output_odom_topic").perform(context)
    if not output_odom_topic:
        output_odom_topic = "odometry/filtered_enu"

    output_ned_odom_topic = LaunchConfiguration("output_ned_odom_topic").perform(context)
    if not output_ned_odom_topic:
        output_ned_odom_topic = "odometry/filtered"

    ekf_overrides = {
        "map_frame": map_frame,
        "odom_frame": odom_frame,
        "base_link_frame": base_link_frame,
        "world_frame": world_frame,
        "publish_tf": LaunchConfiguration("publish_tf"),
    }

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_world_enu",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "3.14159265359",
                "--pitch", "0.0",
                "--yaw", "1.57079632679",
                "--frame-id", "world_ned",
                "--child-frame-id", world_frame,
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="log",
            parameters=[ekf_params, ekf_overrides],
            remappings=[
                ("odometry/filtered", output_odom_topic),
            ],
        ),
    ]

    if bool_launch_arg(context, "use_navsat"):
        datum_latitude = float(LaunchConfiguration("datum_latitude").perform(context))
        datum_longitude = float(LaunchConfiguration("datum_longitude").perform(context))
        datum_heading = float(LaunchConfiguration("datum_heading").perform(context))

        nodes.append(
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[
                    navsat_params,
                    {
                        "wait_for_datum": bool_launch_arg(context, "wait_for_datum"),
                        "datum": [datum_latitude, datum_longitude, datum_heading],
                    },
                ],
                remappings=[
                    ("gps/fix", "sensors/gps"),
                    ("imu", LaunchConfiguration("imu_enu_topic")),
                    ("odometry/filtered", output_odom_topic),
                    ("odometry/gps", "sensors/gps/odometry"),
                ],
            )
        )

    if bool_launch_arg(context, "convert_imu_ned_to_enu"):
        nodes.append(
            Node(
                package="sura_localization",
                executable="imu_ned_to_enu",
                name="imu_ned_to_enu",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("imu_ned_topic"),
                        "output_topic": LaunchConfiguration("imu_enu_topic"),
                        "output_frame": imu_enu_frame,
                        "convert_frd_to_flu": True,
                        "orientation_yaw_stddev_deg": LaunchConfiguration(
                            "imu_orientation_yaw_stddev_deg"
                        ),
                    }
                ],
            )
        )

    if bool_launch_arg(context, "convert_pressure_to_pose"):
        nodes.append(
            Node(
                package="sura_localization",
                executable="pressure_to_pose",
                name="pressure_to_pose",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("pressure_topic"),
                        "output_topic": LaunchConfiguration("pressure_pose_topic"),
                        "environment": LaunchConfiguration("environment"),
                        "frame_id": world_frame,
                        "sensor_frame_id": namespaced_frame(robot_namespace, "pressure_link"),
                        "positive_down": True,
                        "z_scale": LaunchConfiguration("pressure_pose_z_scale"),
                        "z_offset_m": LaunchConfiguration("pressure_pose_z_offset_m"),
                        "fallback_z_variance": 0.01,
                        "fallback_xy_variance": 0.01,
                    }
                ],
            )
        )

    nodes.append(
        Node(
            package="sura_localization",
            executable="enu_to_ned_odometry",
            name="enu_to_ned_odometry",
            output="screen",
            parameters=[
                {
                    "input_topic": output_odom_topic,
                    "output_topic": output_ned_odom_topic,
                    "frame_id": LaunchConfiguration("ned_world_frame"),
                    "child_frame_id": base_link_frame,
                }
            ],
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value=""),
            DeclareLaunchArgument("environment", default_value="real"),
            DeclareLaunchArgument("config_package", default_value="sura_localization"),
            DeclareLaunchArgument("config_file", default_value="config/auv_localization.yaml"),
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
            DeclareLaunchArgument("imu_orientation_yaw_stddev_deg", default_value="-1.0"),
            DeclareLaunchArgument("convert_pressure_to_pose", default_value="true"),
            DeclareLaunchArgument("pressure_topic", default_value="sensors/pressure"),
            DeclareLaunchArgument("pressure_pose_topic", default_value="sensors/pressure/pose"),
            DeclareLaunchArgument("pressure_pose_z_scale", default_value="1.0"),
            DeclareLaunchArgument("pressure_pose_z_offset_m", default_value="0.0"),
            DeclareLaunchArgument("output_odom_topic", default_value=""),
            DeclareLaunchArgument("output_ned_odom_topic", default_value=""),
            DeclareLaunchArgument("ned_world_frame", default_value="world_ned"),
            OpaqueFunction(function=launch_setup),
        ]
    )
