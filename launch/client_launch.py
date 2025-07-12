from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    
    config = os.path.join(
        '/ros2_ws/',
        'phntm_bridge_params.yaml'
        )
    
    color = LaunchConfiguration("color", default="false")
    gdb_debugger = LaunchConfiguration("debugger", default="false").perform(context)
    gdb_server = LaunchConfiguration("gdb_server", default="false").perform(context)
    gdb_server_port = LaunchConfiguration("gdb_server_port", default="3000").perform(context)
    bridge_launch_prefix = ""
    if gdb_debugger == "true":
       bridge_launch_prefix = "gdb -ex run --args"
    elif gdb_server == "true":
        bridge_launch_prefix = f"gdbserver localhost:{gdb_server_port}"
    
    return [
        Node(
            package='phntm_bridge',
            executable='phntm_bridge',
            prefix=[bridge_launch_prefix],
            emulate_tty=True,
            output='screen',
            parameters=[config],
        )
    ]

def generate_launch_description():
    
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
