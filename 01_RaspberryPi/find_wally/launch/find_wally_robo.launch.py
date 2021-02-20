## リアルウォーリーを探せ一括起動launch
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
        node_name='rplidar_composition',
        package='rplidar_ros',
        node_executable='rplidar_composition',
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

    # ジョイコントローラ用ノード
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    # ジョイコントローラでROSロボを操作するノード
    rosmarica_node = Node(
        package='rosmarica_teleop',
        executable='joystick_node',
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
        arguments=['0', '0', '0.1', '0', '0',
                   '0', 'base_footprint', 'laser'],
    )

    # slam_toolboxノード
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory('find_wally')
            + 'config/mapper_params_offline.yaml'
        ],
    )

    # リアルウォーリーを探せ！ノード
    find_wally_node = Node(
        package='find_wally',
        executable='find_wally',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(ros2arduino_node)
    ld.add_action(v4l2_camera_node)
    ld.add_action(rplider_node)
    ld.add_action(joy_node)
    ld.add_action(rosmarica_node)
    ld.add_action(odom_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(slam_node)
    ld.add_action(find_wally_node)

    return ld