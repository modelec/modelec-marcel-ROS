from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    with_gui_arg = DeclareLaunchArgument('with_gui', default_value='true', description='Launch GUI?')
    with_rplidar_arg = DeclareLaunchArgument('with_rplidar', default_value='true', description='Launch RPLIDAR?')
    with_com_arg = DeclareLaunchArgument('with_com', default_value='true', description='Launch COM nodes?')
    with_strat_arg = DeclareLaunchArgument('with_strat', default_value='true', description='Launch strategy nodes?')

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # ----------------------------
    # Infinite LIDAR restart logic
    # ----------------------------
    def create_lidar_with_restart():
        """Create a lidar node and its auto-restart handler."""
        lidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
            output='screen'
        )

        # Instead of recursion at definition time, we delay the re-creation using a lambda
        restart_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=lidar_node,
                on_exit=[
                    LogInfo(msg='[Launch] RPLIDAR crashed — restarting in 2s...'),
                    TimerAction(
                        period=0.3,
                        actions=[OpaqueFunction(function=lambda *_: create_lidar_with_restart())]
                    )
                ]
            )
        )

        return [lidar_node, restart_handler]

    def launch_rplidar_restart_if_needed(context, *args, **kwargs):
        if context.launch_configurations.get('with_rplidar') == 'true':
            # Start after 3 seconds
            return [
                TimerAction(
                    period=3.0,
                    actions=create_lidar_with_restart()
                )
            ]
        return []

    # ----------------------------
    # GUI node
    # ----------------------------
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

    # ----------------------------
    # COM nodes
    # ----------------------------
    def launch_com(context, *args, **kwargs):
        if context.launch_configurations.get('with_com') == 'true':
            return [
                Node(package='modelec_com', executable='serial_listener', name='serial_listener'),
                Node(
                    package='modelec_com',
                    executable='pcb_odo_interface',
                    name='pcb_odo_interface',
                    parameters=[{
                        'serial_port': "/dev/USB_ODO",
                        'baudrate': 115200,
                        'name': "pcb_odo",
                    }]
                ),
                Node(
                    package='modelec_com',
                    executable='pcb_action_interface',
                    name='pcb_action_interface',
                    parameters=[{
                        'serial_port': "/tmp/USB_ACTION",
                        'baudrate': 115200,
                        'name': "pcb_action",
                    }]
                ),
            ]
        return []

    # ----------------------------
    # Strategy nodes
    # ----------------------------
    def launch_strat(context, *args, **kwargs):
        if context.launch_configurations.get('with_strat') == 'true':
            return [
                Node(package='modelec_strat', executable='strat_fsm', name='strat_fsm'),
                Node(package='modelec_strat', executable='pami_manager', name='pami_manager'),
                Node(package='modelec_strat', executable='enemy_manager', name='enemy_manager'),
            ]
        return []

    # ----------------------------
    # Final launch description
    # ----------------------------
    return LaunchDescription([
        DeclareLaunchArgument('channel_type', default_value=channel_type, description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value=inverted, description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='Specifying scan mode of lidar'),

        with_gui_arg,
        with_rplidar_arg,
        with_com_arg,
        with_strat_arg,

        OpaqueFunction(function=launch_rplidar_restart_if_needed),
        OpaqueFunction(function=launch_gui),
        OpaqueFunction(function=launch_com),
        OpaqueFunction(function=launch_strat),
    ])
