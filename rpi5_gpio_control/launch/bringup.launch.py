from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

import os

def generate_launch_description():
    pkg = get_package_share_directory('rpi5_gpio_control')
    urdf = os.path.join(pkg, 'urdf', 'robot.urdf')
    yaml = os.path.join(pkg, 'config', 'valve_controller.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf).read()}],
    )

    cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[yaml]
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    valves = Node(
        package='controller_manager',
    executable='spawner',
    arguments=[
        'valve_controller',
        '--controller-manager', '/controller_manager',
        '--param-file', yaml
    ],
    output='screen',
    )

    # Orden: cm después de rsp; jsb después de cm; valves un instante después de jsb
    after_cm = RegisterEventHandler(
        OnProcessStart(target_action=cm, on_start=[jsb])
    )
    after_jsb = RegisterEventHandler(
        OnProcessStart(target_action=jsb, on_start=[TimerAction(period=0.5, actions=[valves])])
    )  

    return LaunchDescription([rsp,cm, after_cm, after_jsb])
