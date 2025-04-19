from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch.substitutions import EnvironmentVariable
import pathlib
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Set to false for real‑life operation'
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('jaeger_model'),
                    'config',
                    'ekf.yaml'
                ),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])
