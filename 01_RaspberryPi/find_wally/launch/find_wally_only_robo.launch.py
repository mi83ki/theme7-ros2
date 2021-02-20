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

    # yolo v3 ノード
    darknet_ros_share_dir = get_package_share_directory('darknet_ros')
    #image = LaunchConfiguration('image', default = '/camera/rgb/image_raw')
    image = LaunchConfiguration('image', default = '/image_raw')
    yolo_weights_path = LaunchConfiguration('yolo_weights_path', default = darknet_ros_share_dir + '/yolo_network_config/weights')
    yolo_config_path = LaunchConfiguration('yolo_config_path', default = darknet_ros_share_dir + '/yolo_network_config/cfg')
    ros_param_file = LaunchConfiguration('ros_param_file', default = darknet_ros_share_dir + 'config/ros.yaml')
    #network_param_file = LaunchConfiguration('network_param_file', default = darknet_ros_share_dir + 'config/yolov2-tiny.yaml')
    network_param_file = LaunchConfiguration('network_param_file', default = darknet_ros_share_dir + 'config/yolov3-tiny.yaml')
    declare_image_cmd = DeclareLaunchArgument(
        'image',
        #default_value = '/camera/rgb/image_raw',
        default_value = '/image_raw',
        description = 'Image topic')
    declare_yolo_weights_path_cmd = DeclareLaunchArgument(
        'yolo_weights_path',
        default_value = darknet_ros_share_dir + '/yolo_network_config/weights',
        description = 'Path to yolo weights') 
    declare_yolo_config_path_cmd = DeclareLaunchArgument(
        'yolo_config_path',
        default_value = darknet_ros_share_dir + '/yolo_network_config/cfg',
        description = 'Path to yolo config') 
    declare_ros_param_file_cmd = DeclareLaunchArgument(
        'ros_param_file',
        default_value = darknet_ros_share_dir + '/config/ros.yaml',
        description = 'Path to file with ROS related config')  
    declare_network_param_file_cmd = DeclareLaunchArgument(
        'network_param_file',
        #default_value = darknet_ros_share_dir + '/config/yolov2-tiny.yaml',
        default_value = darknet_ros_share_dir + '/config/yolov3-tiny.yaml',
        description = 'Path to file with network param file')  
    darknet_ros_cmd = Node(
        package='darknet_ros',
        node_executable='darknet_ros',
        node_name='darknet_ros',
        output='screen',
        parameters=[ros_param_file, network_param_file,
        {
            "config_path": yolo_config_path, 
            "weights_path": yolo_weights_path,
        },
        ])

    # リアルウォーリーを探せ！ノード
    find_wally_node = Node(
        package='find_wally',
        executable='find_wally',
        output='screen',
    )

    # rviz2ノード
    rviz2_node = Node(
        node_name='rviz2',
        package='rviz2',
        node_executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            get_package_share_directory('find_wally')
            + '/config/default.rviz'],
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

    ld.add_action(declare_image_cmd)
    ld.add_action(declare_yolo_weights_path_cmd)
    ld.add_action(declare_yolo_config_path_cmd)
    ld.add_action(declare_ros_param_file_cmd)
    ld.add_action(declare_network_param_file_cmd)
    ld.add_action(darknet_ros_cmd)

    ld.add_action(find_wally_node)
    ld.add_action(rviz2_node)

    return ld