from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pcd_path_arg = DeclareLaunchArgument(
        "pcd_path",
        default_value="",
        description="Absolute path to map.pcd",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="map",
        description="Point cloud frame id",
    )
    topic_name_arg = DeclareLaunchArgument(
        "topic_name",
        default_value="/pgo/saved_map_cloud",
        description="Published point cloud topic",
    )
    publish_hz_arg = DeclareLaunchArgument(
        "publish_hz",
        default_value="1.0",
        description="Cloud publish rate in Hz",
    )
    voxel_size_arg = DeclareLaunchArgument(
        "voxel_size",
        default_value="0.0",
        description="Optional voxel downsample size in meter, 0 means disabled",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz",
    )

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pgo"), "rviz", "saved_map_viewer.rviz"]
    )

    pcd_map_publisher = Node(
        package="pgo",
        executable="pcd_map_publisher",
        name="pcd_map_publisher",
        output="screen",
        parameters=[
            {"pcd_path": LaunchConfiguration("pcd_path")},
            {"topic_name": LaunchConfiguration("topic_name")},
            {"frame_id": LaunchConfiguration("frame_id")},
            {"publish_hz": LaunchConfiguration("publish_hz")},
            {"voxel_size": LaunchConfiguration("voxel_size")},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="saved_map_rviz",
        output="screen",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            pcd_path_arg,
            frame_id_arg,
            topic_name_arg,
            publish_hz_arg,
            voxel_size_arg,
            use_rviz_arg,
            pcd_map_publisher,
            rviz_node,
        ]
    )
