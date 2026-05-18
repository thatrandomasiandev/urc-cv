from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('urc_localization')
    rtabmap_config = PathJoinSubstitution([pkg_share, 'config', 'rtabmap.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),

        Node(
            package='rtabmap_ros',
            executable='rgbd_odometry',
            name='rtabmap_odometry',
            output='screen',
            parameters=[rtabmap_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/rgb/image', '/camera/color/image_raw'),
                ('/rgb/camera_info', '/camera/color/camera_info'),
                ('/depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),
    ])
