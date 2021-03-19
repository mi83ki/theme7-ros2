## SLAM一括起動launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # airoboの起動
    airobo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('airobo_bringup'), '/airobo_bringup.launch.py']),
    )

    # tf2ノード
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0.1', '0', '3.14',
                   '3.14', 'base_footprint', 'laser'],
    )

    ld = LaunchDescription()
    ld.add_action(airobo_bringup)
    ld.add_action(static_transform_publisher_node)

    return ld