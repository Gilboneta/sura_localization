import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    package_share = get_package_share_directory("sura_localization")
    config_file = os.path.join(package_share, "config", "ekf_surface_fastlio.yaml")

    map_frame = LaunchConfiguration("map_frame")
    publish_tf = LaunchConfiguration("publish_tf")

    base_link_frame = LaunchConfiguration("base_link_frame").perform(context)
    if not base_link_frame:
        base_link_frame = f"{robot_namespace}/base_link_enu"
    map_frame_value = map_frame.perform(context)
    if not map_frame_value:
        map_frame_value = f"{robot_namespace}/map"
    odom_frame_value = LaunchConfiguration("ekf_odom_frame").perform(context)
    if not odom_frame_value:
        odom_frame_value = f"{robot_namespace}/odom"
    world_frame_value = LaunchConfiguration("ekf_world_frame").perform(context)
    if not world_frame_value:
        world_frame_value = map_frame_value
    datum_latitude = float(LaunchConfiguration("datum_latitude").perform(context))
    datum_longitude = float(LaunchConfiguration("datum_longitude").perform(context))
    datum_heading = float(LaunchConfiguration("datum_heading").perform(context))

    ekf_overrides = {
        "map_frame": map_frame_value,
        "odom_frame": odom_frame_value,
        "base_link_frame": base_link_frame,
        "world_frame": world_frame_value,
        "publish_tf": publish_tf,
        "odom0": "fastlio/odometry",
        # "odom1": "sensors/gps/odometry",
    }
    gps_anchor_overrides = {
        "gps_topic": "sensors/gps/fix",
        "fastlio_map_frame": map_frame_value,
        "position_frame": f"{robot_namespace}/gps_frame",
        "base_frame": base_link_frame,
    }

    return [
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
                "--child-frame-id", "world_enu",
            ],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "wait_for_datum": True,
                    "datum": [datum_latitude, datum_longitude, datum_heading],
                },
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_file, ekf_overrides],
        ),
        Node(
            package="sura_sensors",
            executable="gps_anchor_node",
            name="gps_anchor_node",
            output="screen",
            parameters=[config_file, gps_anchor_overrides],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace"),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("ekf_odom_frame", default_value=""),
            DeclareLaunchArgument("base_link_frame", default_value=""),
            DeclareLaunchArgument("ekf_world_frame", default_value=""),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("datum_latitude"),
            DeclareLaunchArgument("datum_longitude"),
            DeclareLaunchArgument("datum_heading"),
            OpaqueFunction(function=launch_setup),
        ]
    )
