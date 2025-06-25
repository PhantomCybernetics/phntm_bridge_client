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
    
    use_gdb_server = LaunchConfiguration("use_gdb_server", default="false")
    launch_prefix = ""
    if use_gdb_server.perform(context) == "true":
        launch_prefix += "gdbserver localhost:3000"
    
    return [
        Node(
            package='phntm_bridge',
            executable='phntm_bridge',
            prefix=[launch_prefix],
            emulate_tty=True,
            output='screen',
            parameters=[config],
        )
    ]

def generate_launch_description():
    
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
