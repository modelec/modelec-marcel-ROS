from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    with_gui_arg = DeclareLaunchArgument(
        'with_gui',
        default_value='true',
        description='Whether to launch the GUI'
    )

    with_rplidar_arg = DeclareLaunchArgument(
        'with_rplidar',
        default_value='true',
        description='Whether to launch the RPLIDAR'
    )

    with_gui = LaunchConfiguration('with_gui')
    with_rplidar = LaunchConfiguration('with_rplidar')

    # GUI node
    gui_node = Node(
        package='modelec_gui',
        executable='modelec_gui',
        name='modelec_gui',
        condition=IfCondition(with_gui)
    )

    shutdown_on_gui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gui_node,
            on_exit=[Shutdown()],
            condition=IfCondition(with_gui)
        )
    )

    # RPLIDAR node
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

    return LaunchDescription([
        with_gui_arg,
        with_rplidar_arg,
        rplidar_launch,
        gui_node,
        shutdown_on_gui_exit,
        Node(package='modelec_com', executable='serial_listener', name='serial_listener'),
        Node(package='modelec_com', executable='pcb_odo_interface', name='pcb_odo_interface'),
        Node(package='modelec_com', executable='pcb_alim_interface', name='pcb_alim_interface'),
        Node(package='modelec_com', executable='pcb_action_interface', name='pcb_action_interface'),
        Node(package='modelec_strat', executable='strat_fsm', name='strat_fsm'),
        Node(package='modelec_strat', executable='pami_manager', name='pami_manager'),
        Node(package='modelec_strat', executable='enemy_manager', name='enemy_manager')
    ])
