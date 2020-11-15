import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='my_ros2serial_python',
            executable='serial_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rosmarica_teleop',
            executable='joystick_node',
            output='screen'
        ),
    ])