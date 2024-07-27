
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def load_controller(controller_name,):
    return ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                controller_name],
        output='screen',
        emulate_tty=True
    )

def register_loading_order(current_component, next_component, condition=None):
    return RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=current_component,
            on_exit=[next_component],
        ),
        condition=condition
    )

def register_sequential_loading(*components):
    return [register_loading_order(components[i], components[i+1]) for i in range(len(components) - 1)]
