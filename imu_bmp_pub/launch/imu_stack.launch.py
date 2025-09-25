from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

#Launch para probar todo sin control :/

def generate_launch_description():
    pkg_share = get_package_share_directory('imu_bmp_pub')
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf_imu.yaml')

    imu_node = Node(
        package='imu_bmp_pub',
        executable='imu_bmp_node',
        name='imu_bmp_node',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'rate_hz': 100.0,
        }]
    )

    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'gain': 0.1,          # para converger más rápido al parecer pero no me funciona xd
            'zeta': 0.0           # compensación de deriva del gyro
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml]
    )

    # IMU -> base_link 
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link_broadcaster',
        arguments=['0', '0', '0',  '0', '0', '0',  'base_link', 'imu_link']
    )

    pwm_node = Node(
    package='imu_bmp_pub',
    executable='pwm_and_valves',
    name='pwm_and_valves',
    output='screen',
    parameters=[{
        'pwm_pins': [12,13],
        'pwm_freq': 50.0,
        'pwm_duty_pct': 5.0,
        'valve_pins': [17,27,22,23,24],
        'toggle_period': 0,   # 0 para desactivar el toggle automático
        'start_mask': 0
    }]
)


    return LaunchDescription([pwm_node,imu_node,madgwick,ekf,static_tf])
