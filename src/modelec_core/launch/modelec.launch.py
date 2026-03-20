import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # -------------------------------------------------
    # Launch arguments
    # -------------------------------------------------
    with_gui_arg = DeclareLaunchArgument('with_gui', default_value='true')
    with_rplidar_arg = DeclareLaunchArgument('with_rplidar', default_value='true')
    with_com_arg = DeclareLaunchArgument('with_com', default_value='true')
    with_strat_arg = DeclareLaunchArgument('with_strat', default_value='true')
    with_enemy_manager_arg = DeclareLaunchArgument('with_enemy_manager', default_value='true')
    with_joy_arg = DeclareLaunchArgument('with_joy', default_value='true')
    with_color_detector_arg = DeclareLaunchArgument('with_color_detector', default_value='true')

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # -------------------------------------------------
    # RPLIDAR logic (Starts Immediately)
    # -------------------------------------------------
    def launch_lidar(context, *args, **kwargs):
        if context.launch_configurations.get('with_rplidar') == 'true':
            lidar_node = Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                parameters=[{'channel_type':channel_type,
                             'serial_port': serial_port,
                             'serial_baudrate': serial_baudrate,
                             'frame_id': frame_id,
                             'inverted': inverted,
                             'angle_compensate': angle_compensate,
                             'scan_mode': scan_mode}]
            )
            return [lidar_node]
        return []

    # -------------------------------------------------
    # Node Launch Functions
    # -------------------------------------------------
    def launch_gui(context, *args, **kwargs):
        if context.launch_configurations.get('with_gui') == 'true':
            gui = Node(package='modelec_gui', executable='modelec_gui', name='modelec_gui')
            return [gui, RegisterEventHandler(OnProcessExit(target_action=gui, on_exit=[Shutdown()]))]
        return []

    def launch_com(context, *args, **kwargs):
        if context.launch_configurations.get('with_com') == 'true':
            return [
                Node(package='modelec_com', executable='pcb_odo_interface', name='pcb_odo_interface'),
                Node(package='modelec_com', executable='pcb_action_interface', name='pcb_action_interface'),
            ]
        return []

    def launch_strat(context, *args, **kwargs):
        if context.launch_configurations.get('with_strat') == 'true':
            return [
                Node(package='modelec_strat', executable='strat_fsm', name='strat_fsm'),
                Node(package='modelec_strat', executable='pami_manager', name='pami_manager'),
            ]
        return []

    def launch_enemy_manager(context, *args, **kwargs):
        if context.launch_configurations.get('with_enemy_manager') == 'true':
            return [Node(package='modelec_strat', executable='enemy_manager', name='enemy_manager')]
        return []

    def launch_joy(context, *args, **kwargs):
        if context.launch_configurations.get('with_joy') == 'true':
            return [Node(package='joy', executable='joy_node', name='joy_node')]
        return []

    def launch_color_detector(context, *args, **kwargs):
        if context.launch_configurations.get('with_color_detector') == 'true':
            return [Node(package='modelec_com', executable='color_detector', name='color_detector')]
        return []

    # -------------------------------------------------
    # Final LaunchDescription
    # -------------------------------------------------
    return LaunchDescription([
        with_gui_arg, with_rplidar_arg, with_com_arg, with_strat_arg,
        with_enemy_manager_arg, with_joy_arg, with_color_detector_arg,

        OpaqueFunction(function=launch_lidar),
        OpaqueFunction(function=launch_gui),
        OpaqueFunction(function=launch_com),
        OpaqueFunction(function=launch_strat),
        OpaqueFunction(function=launch_enemy_manager),
        OpaqueFunction(function=launch_joy),
        OpaqueFunction(function=launch_color_detector),
    ])