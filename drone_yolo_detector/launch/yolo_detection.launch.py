from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    model_path = PathJoinSubstitution([
        FindPackageShare('drone_yolo_detector'),
        'drone_yolo_detector',
        'model',
        'best.pt'
    ])

    return LaunchDescription([
        Node(
            package='drone_yolo_detector',
            executable='yolo_node',
            name='yolo_detector',
            output='screen',
            parameters=[{'model_path': 'model/best.pt'}]
        )
    ])
