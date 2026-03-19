import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------------------------------
    # Launch arguments
    # -------------------------------------------------
    args = [
        DeclareLaunchArgument('with_gui', default_value='true'),
        DeclareLaunchArgument('with_rplidar', default_value='true'),
        DeclareLaunchArgument('with_com', default_value='true'),
        DeclareLaunchArgument('with_strat', default_value='true'),
        DeclareLaunchArgument('with_enemy_manager', default_value='true'),
        DeclareLaunchArgument('with_joy', default_value='true'),
        DeclareLaunchArgument('with_color_detector', default_value='true'),

        # RPLidar specific arguments
        DeclareLaunchArgument('channel_type', default_value='serial'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity'),
    ]

    # -------------------------------------------------
    # Nodes Configuration
    # -------------------------------------------------

    # RPLIDAR Node
    rplidar_node = Node(
        condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_rplidar')),
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': LaunchConfiguration('channel_type'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': LaunchConfiguration('inverted'),
            'angle_compensate': LaunchConfiguration('angle_compensate'),
            'scan_mode': LaunchConfiguration('scan_mode')
        }]
    )

    # GUI Node (with Shutdown on exit)
    gui_node = Node(
        condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_gui')),
        package='modelec_gui',
        executable='modelec_gui',
        name='modelec_gui',
        on_exit=Shutdown()
    )

    # Communication Nodes
    com_nodes = [
        Node(
            condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_com')),
            package='modelec_com', executable='pcb_odo_interface', name='pcb_odo_interface'
        ),
        Node(
            condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_com')),
            package='modelec_com', executable='pcb_action_interface', name='pcb_action_interface'
        ),
    ]

    # Strategy Nodes
    strat_nodes = [
        Node(
            condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_strat')),
            package='modelec_strat', executable='strat_fsm', name='strat_fsm'
        ),
        Node(
            condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_strat')),
            package='modelec_strat', executable='pami_manager', name='pami_manager'
        ),
    ]

    # Enemy Manager
    enemy_manager_node = Node(
        condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_enemy_manager')),
        package='modelec_strat', executable='enemy_manager', name='enemy_manager'
    )

    # Joystick
    joy_node = Node(
        condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_joy')),
        package='joy', executable='joy_node', name='joy_node'
    )

    # Color Detector
    color_detector_node = Node(
        condition=launch_ros.conditions.IfCondition(LaunchConfiguration('with_color_detector')),
        package='modelec_com', executable='color_detector', name='color_detector'
    )

    # -------------------------------------------------
    # Final LaunchDescription
    # -------------------------------------------------
    return LaunchDescription(args + [
        rplidar_node,
        gui_node,
        *com_nodes,
        *strat_nodes,
        enemy_manager_node,
        joy_node,
        color_detector_node
    ])