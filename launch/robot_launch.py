from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths
    test_robot_dir = '/home/mickey/ros2_ws/src/test_robot'
    nav2_launch_dir = get_package_share_directory('nav2_bringup')
    realsense_dir = get_package_share_directory('realsense2_camera')
    nav2_bt_dir = get_package_share_directory('nav2_bt_navigator')

    nav2_params_path = os.path.join(test_robot_dir, 'config', 'nav2_params.yaml')
    map_yaml_path = os.path.join(test_robot_dir, 'maps', 'map.yaml')
    rviz_config_path = os.path.join(test_robot_dir, 'config', 'nav2.rviz')
    nav2_bt_path = os.path.join(nav2_bt_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    return LaunchDescription([
        # TF Static transforms
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            name="map_to_odom_tf"
        ),

        # Hoverboard driver
        Node(
            package='hoverboard_driver_usart2',
            executable='hoverboard_node',
            name='hoverboard_driver',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'publish_tf': True,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint'
            }]
        ),

        # Additional TF transforms
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_footprint", "base_link"],
            name="base_footprint_to_base_tf"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.1", "0", "0.05", "0", "0", "0", "base_link", "camera_link"],
            name="base_to_camera_tf"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "camera_link", "camera_depth_frame"],
            name="camera_link_to_depth_frame_tf"
        ),

        # Delayed start of navigation components
        TimerAction(
            period=5.0,
            actions=[
                # RealSense Camera
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(realsense_dir, 'launch', 'rs_launch.py')
                    ),
                    launch_arguments={
                        'depth_module.profile': '640x480x15',
                        'enable_depth': 'true',
                        'align_depth.enable': 'true',
                        'depth_format': 'Z16',
                        'infra_width': '640',
                        'infra_height': '480',
                        'infra_fps': '15',
                        'enable_imu': 'false',
                        'initial_reset': 'true',  # Add hardware reset on start
                        'reconnect_timeout': '10.0'  # Add reconnect timeout
                    }.items()
                ),

                # Depth to LaserScan
                Node(
                    package='depthimage_to_laserscan',
                    executable='depthimage_to_laserscan_node',
                    name='depth_to_laserscan',
                    parameters=[{
                        'output_frame': 'camera_depth_frame',
                        'scan_height': 10,
                        'range_min': 0.2,
                        'range_max': 5.0,
                        'scan_time': 0.033,
                        'angle_min': -0.87,
                        'angle_max': 0.87,
                        'angle_increment': 0.0087,
                        'qos_overrides./scan.publisher.reliability': 'reliable',
                        'qos_overrides./scan.publisher.durability': 'volatile'
                    }],
                    remappings=[
                        ('depth', '/camera/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/camera/depth/camera_info'),
                        ('scan', '/scan')
                    ]
                ),

                # Map Server
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{
                        'yaml_filename': map_yaml_path,
                        'use_sim_time': False,
                        'frame_id': 'map',
                        'topic_name': 'map',
                        'publish_period_sec': 0.1,
                        'always_send_full_costmap': True,
                        'subscribe_to_updates': True,
                        'free_thresh': 0.25,
                        'occupied_thresh': 0.65,
                        # Add these parameters
                        'map_subscribe_transient_local': True,
                        'transform_tolerance': 0.1
                    }],
                                # Add to map server node
                    remappings=[
                        ('map', '/map'),
                        ('map_updates', '/map_updates')
                    ]
                ),

                # AMCL
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[{
                        'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
                        'set_initial_pose': True,
                        'use_sim_time': False,
                        'alpha1': 0.2,
                        'alpha2': 0.2,
                        'alpha3': 0.2,
                        'alpha4': 0.2,
                        'max_particles': 2000,
                        'laser_max_range': 5.0,
                        'laser_min_range': 0.3,
                        'laser_model_type': 'likelihood_field',
                        'odom_frame_id': 'odom',
                        'base_frame_id': 'base_footprint',
                        'global_frame_id': 'map',
                        'transform_tolerance': 1.0
                    }]
                ),

                # Lifecycle Manager
                #Node(
                #    package='nav2_lifecycle_manager',
                #    executable='lifecycle_manager',
                #    name='lifecycle_manager_navigation',
                #    output='screen',
                #    parameters=[{
                #        'use_sim_time': False,
                #        'autostart': True,
                #        'node_names': ['map_server', 'amcl']
                #    }]
                #),
                # Map Server Lifecycle Manager
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['map_server']
                    }]
                ),
                # Navigation Stack
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_launch_dir, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'params_file': nav2_params_path,
                        'bt_xml_file': nav2_bt_path
                    }.items()
                ),

                # RViz2
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_path],
                    parameters=[{
                        'use_sim_time': False
                    }]
                )
            ]
        )
    ])