## airoboの起動launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'airobo_description.urdf'
    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('airobo_description'),
        'urdf',
        urdf_file_name)

    arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

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
    # base_footprintからlaserに変換
    base_footprint_to_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        name='base_footprint_to_laser',
        arguments=['0', '0', '0.11','0', '3.1415', '3.1415',
                    'base_footprint', 'laser'],
    )
    # base_footprintからbase_linkに変換
    #base_footprint_to_base_link_node = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    output='screen',
    #    name='base_footprint_to_base_link',
    #    arguments=['0', '0', '0.045', '0', '0', '0',
    #                'base_footprint', 'base_link'],
    #)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf],
    )

    ld = LaunchDescription()
    ld.add_action(arg)
    ld.add_action(ros2arduino_node)
    ld.add_action(v4l2_camera_node)
    ld.add_action(rplider_node)
    ld.add_action(keyboard_node)
    ld.add_action(odom_node)
    ld.add_action(base_footprint_to_laser_node)
    #ld.add_action(base_footprint_to_base_link_node)
    ld.add_action(robot_state_publisher_node)

    return ld