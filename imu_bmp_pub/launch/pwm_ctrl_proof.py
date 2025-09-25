from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#launch prueba para probar control con odometría falsa del simulador del otro paquete con gazebo XD

def generate_launch_description():
    pkg_share = get_package_share_directory('imu_bmp_pub')
    pkg_control = get_package_share_directory('jaeger_model')

    pwm_node = Node(
        package='imu_bmp_pub',
        executable='dual_pwm_sweep',
        name='dual_pwm_sweep',
        output='screen',
        parameters=[{
            'pwm_pins': [12,13],
            'pwm_freq': 50.0,
            # pin 12
            'p12_sweep_on_start': True,
            'p12_sweep_min_pct': 4.0,
            'p12_sweep_max_pct': 6.1,
            'p12_sweep_step_pct': 0.1,
            'p12_sweep_duration_s': 5.0,
            # pin 13
            'p13_sweep_on_start': True,
            'p13_sweep_min_pct': 4.0,
            'p13_sweep_max_pct': 6.3,
            'p13_sweep_step_pct': 0.1,
            'p13_sweep_duration_s': 5.0,
        }]
    )

    control_launch = os.path.join(pkg_control, 'launch', 'basic_jaeger.launch.py')
    include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )


    return LaunchDescription([pwm_node, include_launch])
