from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_serial_bridge',
            namespace='serial_bridge',
            executable='bridge_test_cmds',
            name='bridge_test_cmds',
            output="screen"
        ),
    ])