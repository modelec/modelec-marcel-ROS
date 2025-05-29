from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    with_gui_arg = DeclareLaunchArgument('with_gui', default_value='true', description='Launch GUI?')
    with_rplidar_arg = DeclareLaunchArgument('with_rplidar', default_value='true', description='Launch RPLIDAR?')
    with_com_arg = DeclareLaunchArgument('with_com', default_value='true', description='Launch COM nodes?')
    with_strat_arg = DeclareLaunchArgument('with_strat', default_value='true', description='Launch strategy nodes?')

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # Get launch configs
    with_gui = LaunchConfiguration('with_gui')
    with_rplidar = LaunchConfiguration('with_rplidar')
    with_com = LaunchConfiguration('with_com')
    with_strat = LaunchConfiguration('with_strat')

    def launch_rplidar_restart_if_needed(context, *args, **kwargs):
        if context.launch_configurations.get('with_rplidar') == 'true':
            def create_rplidar_node():
                return Node(
                    package='rplidar_ros',
                    executable='rplidar_node',
                    name='rplidar_node',
                    parameters=[{
                        'channel_type': channel_type,
                        'serial_port': serial_port,
                        'serial_baudrate': serial_baudrate,
                        'frame_id': frame_id,
                        'inverted': inverted,
                        'angle_compensate': angle_compensate
                    }],
                    output='screen'
                )

            rplidar_node = create_rplidar_node()

            restart_handler = RegisterEventHandler(
                OnProcessExit(
                    target_action=rplidar_node,
                    on_exit=[
                        TimerAction(
                            period=5.0,
                            actions=[create_rplidar_node()]  # ✅ créer un NOUVEAU Node
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
                # Node(package='modelec_com', executable='pcb_odo_interface', name='pcb_odo_interface'),
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

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        with_gui_arg,
        with_rplidar_arg,
        with_com_arg,
        with_strat_arg,

        OpaqueFunction(function=launch_rplidar_restart_if_needed),
        OpaqueFunction(function=launch_gui),
        OpaqueFunction(function=launch_com),
        OpaqueFunction(function=launch_strat),
    ])
