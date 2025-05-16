from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown, RegisterEventHandler
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

    # Get launch configs
    with_gui = LaunchConfiguration('with_gui')
    with_rplidar = LaunchConfiguration('with_rplidar')
    with_com = LaunchConfiguration('with_com')
    with_strat = LaunchConfiguration('with_strat')

    # Conditionally include RPLIDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        ),
        condition=IfCondition(with_rplidar)
    )

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
                Node(package='modelec_com', executable='pcb_alim_interface', name='pcb_alim_interface'),
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

        rplidar_launch,
        OpaqueFunction(function=launch_gui),
        OpaqueFunction(function=launch_com),
        OpaqueFunction(function=launch_strat),
    ])
