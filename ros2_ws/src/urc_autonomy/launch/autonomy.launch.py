from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urc_autonomy_share = FindPackageShare('urc_autonomy')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_realsense = LaunchConfiguration('launch_realsense')
    object_detector_device = LaunchConfiguration('object_detector_device')

    twist_mux_config = PathJoinSubstitution(
        [urc_autonomy_share, 'config', 'twist_mux.yaml']
    )
    ekf_config = PathJoinSubstitution(
        [urc_autonomy_share, 'config', 'ekf.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'launch_realsense',
            default_value='true',
            description='Launch realsense2_camera (disable in Gazebo sim)',
        ),
        DeclareLaunchArgument(
            'object_detector_device',
            default_value='cuda',
            description='Torch device for object_detector_node (use cpu in sim on Mac)',
        ),

        LogInfo(msg='URC 2026 Autonomy Stack launching...'),

        Node(
            package='urc_autonomy',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            parameters=[{'log_dir': '~/urc_logs'}],
        ),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            condition=IfCondition(launch_realsense),
            parameters=[{
                'use_sim_time': use_sim_time,
                'enable_color': True,
                'enable_depth': True,
                'color_width': 1280,
                'color_height': 720,
                'color_fps': 30,
                'depth_width': 848,
                'depth_height': 480,
                'depth_fps': 30,
                'align_depth.enable': True,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urc_localization'),
                    'launch',
                    'visual_odom.launch.py',
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        Node(
            package='urc_localization',
            executable='gps_odom_node',
            name='gps_odom_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'gps_covariance_m2': 5.0,
            }],
        ),

        Node(
            package='aruco_detector',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_clahe': True,
                'confirm_frames': 3,
                'loss_frames': 5,
            }],
        ),

        Node(
            package='urc_autonomy',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'search_timeout_s': 45.0,
                'total_mission_timeout_s': 600.0,
                'stop_distance_m': 1.8,
            }],
        ),

        Node(
            package='urc_autonomy',
            executable='estop_node',
            name='estop_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'heartbeat_timeout_s': 3.0,
                'min_marker_distance_m': 0.5,
            }],
        ),

        Node(
            package='visual_servo',
            executable='servo_node',
            name='servo_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'kp_bearing': 0.03,
                'ki_bearing': 0.0005,
                'kd_bearing': 0.008,
                'stop_distance_m': 1.8,
            }],
        ),

        Node(
            package='search_behavior',
            executable='search_node',
            name='search_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'arc_radius_m': 4.0,
                'arc_omega_rad': 0.12,
            }],
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'cmd_vel_topic': '/cmd_vel_nav2',
                'params_file': PathJoinSubstitution(
                    [urc_autonomy_share, 'config', 'nav2_params.yaml']
                ),
            }.items(),
        ),

        Node(
            package='object_detector',
            executable='object_detector_node',
            name='object_detector_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': 'models/urc_objects.pt',
                'confidence_threshold': 0.45,
                'device': object_detector_device,
                'publish_rate_hz': 15.0,
            }],
        ),

        Node(
            package='object_approach',
            executable='approach_node',
            name='approach_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_class': 'mallet',
                'stop_distance_m': 0.8,
                'min_confidence': 0.55,
            }],
        ),
    ])
