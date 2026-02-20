# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("fanuc_mqtt_bridge"), "config", "bridge_config.yaml"]
        ),
        description="Path to fanuc_mqtt_bridge YAML config file.",
    )

    node = Node(
        package="fanuc_mqtt_bridge",
        executable="fanuc_mqtt_bridge",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
    )

    return LaunchDescription([config_file_arg, node])
