from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_serial_bridge',
            namespace='serial_bridge',
            executable='bridge',
            name='bridge',
            output="screen"
        ),
        Node(
            package='cpp_serial_bridge',
            namespace='serial_bridge',
            executable='publisher_test_cmds',
            name='test_cmds',
            output="screen"
        )
    ])