import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_ros2serial_python',
            executable='serial_node',
            output='screen',
            name='serial_node'
        ),
        launch_ros.actions.Node(
            package='teleop_keyboard',
            executable='teleop_keyboard',
            output='screen',
            name='teleop_keyboard'
        )
    ])
