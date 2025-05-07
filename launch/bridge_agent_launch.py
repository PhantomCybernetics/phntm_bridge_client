from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import os

from launch.actions import (EmitEvent, LogInfo, RegisterEventHandler)
from launch.event_handlers import (OnProcessExit)
from launch.events import Shutdown

def launch_setup(context, *args, **kwargs):

    bridge_config = os.path.join(
        '/ros2_ws/',
        'phntm_bridge_params.yaml'
        )

    bridge_use_gdb_server = LaunchConfiguration("use_gdb_server", default="false")
    bridge_launch_prefix = ""
    if bridge_use_gdb_server.perform(context) == "true":
        bridge_launch_prefix += "gdbserver localhost:3000"

    bridge_node = Node(
        package='phntm_bridge',
        executable='phntm_bridge',
        name='phntm_bridge',
        prefix=[bridge_launch_prefix],
        output='screen',
        emulate_tty=True,
        parameters=[bridge_config]
    )
    
    agent_config = os.path.join(
        '/ros2_ws/',
        'phntm_agent_params.yaml'
        )
    
    agent_node = Node(
            package='phntm_agent',
            executable='agent',
            name='phntm_agent',
            output='screen',
            emulate_tty=True,
            parameters=[agent_config]
        )
    
    return [
        bridge_node,
        agent_node,
       
        RegisterEventHandler(
            OnProcessExit(
                target_action=bridge_node,
                on_exit=[
                    LogInfo(msg='Bridge Node stopped; killing Agent'), # this will restart docker container (e.g. after first run checks)
                    EmitEvent(event=Shutdown(reason='Bridge node exited'))
                ]
            )
        )
    ]
    
def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])