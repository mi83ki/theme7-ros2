## airoboの起動launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # arduinoとの通信ノード（ROSロボ実行必須）
    ros2arduino_node = Node(
        package='my_ros2serial_python',
        executable='serial_node',
        output='screen',
    )

    # カメラ用ノード（ROSロボ実行必須）
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
    )

    # ライダー用ノード（ROSロボ実行必須）
    rplider_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # キーボードでROSロボを操作するノード
    keyboard_node = Node(
        package='teleop_keyboard',
        executable='teleop_keyboard',
        output='screen',
    )

    # オドメトリ計算ノード
    odom_node = Node(
        package='odm_cal',
        executable='odm_cal',
        output='screen',
    )

    # tf2ノード
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', output='screen',
        arguments=['0', '0', '0.1', '0', '3.14',
                   '3.14', 'base_footprint', 'laser'],
    )

    ld = LaunchDescription()
    ld.add_action(ros2arduino_node)
    ld.add_action(v4l2_camera_node)
    ld.add_action(rplider_node)
    ld.add_action(keyboard_node)
    ld.add_action(odom_node)
    ld.add_action(static_transform_publisher_node)

    return ld