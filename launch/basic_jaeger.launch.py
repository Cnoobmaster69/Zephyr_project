import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from xacro import process_file
from nav2_common.launch import ReplaceString
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart

ARGUMENTS = [
    DeclareLaunchArgument('world_name', default_value='empty.sdf', description='Name of the world to load. Match with map if using Nav2.'),
    DeclareLaunchArgument('ros_bridge', default_value='True', description='Run ROS bridge node.'),
    DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='Initial x pose of rasbot in the simulation.'),
    DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='Initial y pose of rasbot in the simulation.'),
    DeclareLaunchArgument('initial_pose_z', default_value='0.1', description='Initial z pose of rasbot in the simulation.'),
    DeclareLaunchArgument('initial_pose_yaw', default_value='0.0', description='Initial yaw pose of rasbot in the simulation.'),
    DeclareLaunchArgument('robot_description_topic', default_value='robot_description', description='Robot description topic.'),
    DeclareLaunchArgument('rsp_frequency', default_value='30.0', description='Robot State Publisher frequency.'),
    DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument('entity', default_value='jaeger', description='Name of the robot'),
    DeclareLaunchArgument('robot_description_topic', default_value='robot_description', description='Robot description topic.'),
]

def get_robot_description():
    pkg_rasbot_gazebo = get_package_share_directory('jaeger_model')
    pkg_rasbot_description = get_package_share_directory('jaeger_model')
    robot_description_path = os.path.join(pkg_rasbot_gazebo, 'urdf', 'jaeger_compl.urdf.xacro')
    mappings = {}
    robot_description_config = process_file(robot_description_path, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    robot_desc = robot_desc.replace(
        'package://rasbot_description/', f'file://{pkg_rasbot_description}/'
    )
    return robot_desc

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rasbot_gazebo = get_package_share_directory('jaeger_model')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    world_name = LaunchConfiguration('world_name')
    ros_bridge = LaunchConfiguration('ros_bridge')
    world_path = PathJoinSubstitution([pkg_rasbot_gazebo,'worlds',world_name])
    bridge_config_file_path = os.path.join(pkg_rasbot_gazebo, 'config', 'bridge_config.yaml')

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [world_path],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    )
    ld.add_action(
        Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                output='screen',
                namespace='andino_gz_sim',
                condition=IfCondition(ros_bridge),
            ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rsp_frequency = LaunchConfiguration('rsp_frequency')
    




    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'publish_frequency':  rsp_frequency,
                    'robot_description': get_robot_description(),
                }
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )
        
    ld.add_action(robot_state_publisher)

    entity = LaunchConfiguration('entity')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    robot_description_topic = LaunchConfiguration('robot_description_topic')
    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', entity,
                '-topic', robot_description_topic,
                '-x', initial_pose_x,
                '-y', initial_pose_y,
                '-z', initial_pose_z,
                '-R', '0',
                '-P', '0',
                '-Y', initial_pose_yaw,
            ],
            output='screen',
        )
    )
    bridge_config = ReplaceString(
        source_file=bridge_config_file_path,
        replacements={'<entity>': entity},
    )

    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_config
            }],
        )
    )

    # ld.add_action(
    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         arguments=['-d', os.path.join(pkg_rasbot_gazebo, 'rviz', 'robot_config.rviz')],
    #         parameters=[{'use_sim_time': True}],
    #         remappings=[
    #             ('/tf', 'tf'),
    #             ('/tf_static', 'tf_static'),
    #         ],
    #     ),
    # )
    ld.add_action(
        Node(
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
        )
    )



# ----------------------------------------------------------------
    # 1) Nodo ros2_control_node (controller_manager)
    # ----------------------------------------------------------------

    controllers_yaml = os.path.join(
    get_package_share_directory('jaeger_model'),
    'config',
    'controllers.yaml'
)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
        controllers_yaml
        # {'robot_description': get_robot_description()},
        # {'use_sim_time': use_sim_time}
     ],
    )
    # ld.add_action(ros2_control_node)

    joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )
    # ld.add_action(joint_state_broadcaster_spawner)

    myctl_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['my_robot_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    # ld.add_action(myctl_spawner)

    delay_ros2_control_node = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=robot_state_publisher,
        on_start=[ros2_control_node]
    ))
    ld.add_action(delay_ros2_control_node) 

    delay_jsb = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=ros2_control_node,
        on_start=[joint_state_broadcaster_spawner]
    ))
    ld.add_action(delay_jsb)  

    delay_controller = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=joint_state_broadcaster_spawner,
        on_start=[myctl_spawner]
    ))
    ld.add_action(delay_controller)  






    return ld