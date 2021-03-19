## SLAM一括起動launch
#import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # slam_toolboxノード
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory('airobo_slam')
            + '/config/mapper_params_offline.yaml'
        ],
    )

    # rviz2ノード
    rviz2_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            get_package_share_directory('airobo_slam')
            + '/config/default.rviz'],
    )

    # キーボード入力のトピックを立てるノード
    #pub_key_node = Node(
    #    package='teleop_keyboard',
    #    executable='pub_keyboard',
    #    output='screen',
    #)

    ld = LaunchDescription()
    ld.add_action(slam_node)
    ld.add_action(rviz2_node)
    #ld.add_action(pub_key_node)

    return ld