import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    SetParameter,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('urc_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'urc_field.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'sim_bridge.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'sim.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    spawn_rover = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            (
                'set -euo pipefail; '
                'PKG_SHARE="$(ros2 pkg prefix --share urc_simulation)"; '
                'xacro "$PKG_SHARE/urdf/rover.urdf.xacro" -o /tmp/urc_rover.urdf; '
                'gz sdf -p /tmp/urc_rover.urdf > /tmp/urc_rover.sdf; '
                'gz service -s /world/urc_field/create '
                '--reqtype gz.msgs.EntityFactory '
                '--reptype gz.msgs.Boolean '
                '--timeout 30000 '
                '--req "sdf_filename: \\"/tmp/urc_rover.sdf\\", '
                'name: \\"rover\\", pose: {position: {x: 0.0, y: 0.0, z: 0.2}}"'
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock for all nodes',
        ),

        SetParameter(name='use_sim_time', value=True),

        LogInfo(msg='URC simulation launching (Gazebo + autonomy stack)...'),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen',
        ),

        TimerAction(
            period=8.0,
            actions=[spawn_rover],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': True,
            }],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urc_autonomy'),
                    'launch',
                    'autonomy.launch.py',
                ]),
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'launch_realsense': 'false',
                'object_detector_device': 'cpu',
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ])
