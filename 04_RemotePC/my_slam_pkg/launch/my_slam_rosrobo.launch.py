
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            '/home/ubuntu/ros2_ws/src/Theme7_ROS2/04_RemotePC/my_slam_pkg/config/mapper_params_offline.yaml'
        ],
    )

    # pub_odom_node = Node(
    #     package='my_slam_pkg',
    #     executable='pub_odom',
    #     output='screen',
    # )

    # rviz2_node = Node(
    #     node_name='rviz2',
    #     package='rviz2', node_executable='rviz2', output='screen',
    #     arguments=[
    #         '-d',
    #         get_package_share_directory('raspimouse_ros2_examples')
    #         + '/config/default.rviz'],
    # )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0.1', '0', '0',
                   '0', 'base_link', 'laser'],
    )

    ld = LaunchDescription()
    ld.add_action(slam_node)
    # ld.add_action(pub_odom_node)
    # ld.add_action(rviz2_node)
    ld.add_action(static_transform_publisher_node)

    return ld