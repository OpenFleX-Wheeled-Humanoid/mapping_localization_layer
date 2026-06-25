import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    core_params_file = LaunchConfiguration("core_params_file")
    bridge_params_file = LaunchConfiguration("bridge_params_file")

    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="fast_livo_core",
        parameters=[core_params_file],
        remappings=[
            # Keep FAST-LIVO2 core out of namespace because namespaced mode
            # triggers a runtime crash in upstream fastlivo_mapping.
            ("/cloud_registered", "/fastlio2/raw/world_cloud"),
            ("/aft_mapped_to_init", "/fastlio2/raw/odom"),
            ("/path", "/fastlio2/raw/path"),
        ],
        output="screen",
    )

    bridge_node = Node(
        package="fast_livo",
        executable="lio_bridge_node",
        namespace=namespace,
        name="lio_bridge",
        parameters=[bridge_params_file],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="fastlio2"),
        DeclareLaunchArgument(
            "core_params_file",
            default_value=os.path.join(
                os.path.dirname(__file__), "..", "config", "livo_navigation.yaml"),
        ),
        DeclareLaunchArgument(
            "bridge_params_file",
            default_value=os.path.join(
                os.path.dirname(__file__), "..", "config", "bridge_navigation.yaml"),
        ),
        fast_livo_node,
        bridge_node,
    ])
