from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # Qt GUI node
    gui_node = Node(
        package='modelec_gui',
        executable='modelec_gui',
        name='modelec_gui'
    )

    # Shut down all nodes when GUI exits
    shutdown_on_gui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gui_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        Node(
            package='modelec_com',
            executable='serial_listener',
            name='serial_listener'
        ),
        Node(
            package='modelec_com',
            executable='pcb_odo_interface',
            name='pcb_odo_interface'
        ),
        Node(
            package='modelec_com',
            executable='pcb_alim_interface',
            name='pcb_alim_interface'
        ),
        Node(
            package='modelec_com',
            executable='pcb_action_interface',
            name='pcb_action_interface'
        ),
        gui_node,
        shutdown_on_gui_exit,
        Node(
            package='modelec_core',
            executable='speed_result',
            name='speed_result'
        ),
        Node(
            package='modelec_strat',
            executable='strat_fsm',
            name='strat_fsm'
        ),
        Node(
            package='modelec_strat',
            executable='pami_manager',
            name='pami_manager'
        )
    ])
