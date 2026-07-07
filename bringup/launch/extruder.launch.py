from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("my_robot_ros2_control")

    declared_arguments = [
        DeclareLaunchArgument(
            "extruder_params",
            default_value=PathJoinSubstitution(
                [pkg_share, "config", "extruder_params.yaml"]
            ),
            description="Extruder serial node parameter file",
        ),
    ]

    extruder_node = Node(
        package="my_robot_ros2_control",
        executable="extruder_serial_node",
        name="extruder_serial_node",
        output="screen",
        parameters=[LaunchConfiguration("extruder_params")],
    )

    return LaunchDescription(declared_arguments + [extruder_node])
