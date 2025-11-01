import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from xacro import process_file

def generate_launch_description():


    description_package = "jaeger_model"
    description_xacro   = "urdf/jaeger_robot.urdf.xacro"
    controllers_file = PathJoinSubstitution([
        FindPackageShare("jaeger_model"), "config", "controllers.yaml"
    ])
    ekf_file = PathJoinSubstitution([
        FindPackageShare("imu_bmp_pub"), "config", "ekf_imu.yaml"
    ])


    xacro_path = PathJoinSubstitution([FindPackageShare(description_package), description_xacro])
    robot_description = Command(["xacro ", xacro_path])

    motores= Node(
        package='imu_bmp_pub',
        executable='dual_pwm_sweep',
        name='dual_pwm_sweep',
        output='screen',
    )

    
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_file,
            {'use_sim_time': False}
        ],
    )

    # ---- RSP: publica /robot_description 
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": False}],
    )

    # ---- Controller Manager
    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[controllers_file, {"use_sim_time": False}],
    )

    # ---- Spawners 
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_robot_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delay_rsp = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=motores,
        on_start=[rsp]
    ))

    delay_ros2_control_node = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=rsp,
        on_start=[cm]
    ))
     

    delay_jsb = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=cm,
        on_start=[jsb_spawner]
    ))
      

    delay_controller = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=jsb_spawner,
        on_start=[ctrl_spawner]
    ))
      

    return LaunchDescription([
        #ekf,
        motores,
        rsp,
        #delay_rsp,
        delay_ros2_control_node,
        # cm,
        delay_jsb,
        delay_controller,
    ])