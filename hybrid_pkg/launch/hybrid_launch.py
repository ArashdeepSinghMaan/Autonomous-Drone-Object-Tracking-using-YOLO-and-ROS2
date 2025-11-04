from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hybrid_pkg',
            executable='simple_cpp_node',
            name='cpp_node'
        ),
        Node(
            package='hybrid_pkg',
            executable='simple_py_node.py',
            name='py_node'
        )
    ])
