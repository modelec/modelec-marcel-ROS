from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    with_gui_arg = DeclareLaunchArgument('with_gui', default_value='true', description='Launch GUI?')
    with_rplidar_arg = DeclareLaunchArgument('with_rplidar', default_value='true', description='Launch RPLIDAR?')
    with_com_arg = DeclareLaunchArgument('with_com', default_value='true', description='Launch COM nodes?')
    with_strat_arg = DeclareLaunchArgument('with_strat', default_value='true', description='Launch strategy nodes?')

    # Get launch configs
    with_gui = LaunchConfiguration('with_gui')
    with_rplidar = LaunchConfiguration('with_rplidar')
    with_com = LaunchConfiguration('with_com')
    with_strat = LaunchConfiguration('with_strat')

    # RPLIDAR Node parameters
    rplidar_serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    rplidar_serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    rplidar_frame_id = LaunchConfiguration('frame_id', default='laser')
    rplidar_inverted = LaunchConfiguration('inverted', default='false')
    rplidar_angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    rplidar_scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # Function to launch RPLIDAR Node directly
    def launch_rplidar_node(context, *args, **kwargs):
        if context.launch_configurations.get('with_rplidar') == 'true':
            rplidar_node = Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                parameters=[{
                    'serial_port': rplidar_serial_port,
                    'serial_baudrate': rplidar_serial_baudrate,
                    'frame_id': rplidar_frame_id,
                    'inverted': rplidar_inverted,
                    'angle_compensate': rplidar_angle_compensate,
                    'scan_mode': rplidar_scan_mode
                }],
                output='screen'
            )

            # Register event handler to restart the node if it crashes
            restart_handler = RegisterEventHandler(
                OnProcessExit(
                    target_action=rplidar_node,
                    on_exit=[
                        TimerAction(
                            period=5.0,  # Delay before restarting the node (in seconds)
                            actions=[rplidar_node]  # Restart the RPLIDAR node
                        )
                    ]
                )
            )

            return [rplidar_node, restart_handler]
        return []

    # Function to launch GUI and shutdown handler
    def launch_gui(context, *args, **kwargs):
        if context.launch_configurations.get('with_gui') == 'true':
            gui = Node(
                package='modelec_gui',
                executable='modelec_gui',
                name='modelec_gui'
            )
            shutdown = RegisterEventHandler(
                OnProcessExit(
                    target_action=gui,
                    on_exit=[Shutdown()]
                )
            )
            return [gui, shutdown]
        return []

    # Function to launch COM nodes
    def launch_com(context, *args, **kwargs):
        if context.launch_configurations.get('with_com') == 'true':
            return [
                Node(package='modelec_com', executable='serial_listener', name='serial_listener'),
                Node(package='modelec_com', executable='pcb_odo_interface', name='pcb_odo_interface'),
                Node(package='modelec_com', executable='pcb_action_interface', name='pcb_action_interface'),
            ]
        return []

    # Function to launch strategy nodes
    def launch_strat(context, *args, **kwargs):
        if context.launch_configurations.get('with_strat') == 'true':
            return [
                Node(package='modelec_strat', executable='strat_fsm', name='strat_fsm'),
                Node(package='modelec_strat', executable='pami_manager', name='pami_manager'),
                Node(package='modelec_strat', executable='enemy_manager', name='enemy_manager'),
            ]
        return []

    return LaunchDescription([
        with_gui_arg,
        with_rplidar_arg,
        with_com_arg,
        with_strat_arg,

        OpaqueFunction(function=launch_rplidar_node),
        OpaqueFunction(function=launch_gui),
        OpaqueFunction(function=launch_com),
        OpaqueFunction(function=launch_strat),
    ])
