import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "pgo.rviz"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    fast_livo_share = get_package_share_directory("fast_livo")


    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [fast_livo_share, "/launch/swerve_lio.launch.py"]
                ),
                launch_arguments={
                    "namespace": "fastlio2",
                    "core_params_file": f"{fast_livo_share}/config/livo_mapping.yaml",
                    "bridge_params_file": f"{fast_livo_share}/config/bridge_mapping.yaml",
                }.items(),
            ),
            launch_ros.actions.Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[{"config_path": pgo_config_path}]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg],
            )
        ]
    )
