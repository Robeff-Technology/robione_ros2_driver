import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robione_ros2_driver'),
        'config',
        'robione_ros2_driver.param.yaml'
    )

    return LaunchDescription([
        Node(
            package='robione_ros2_driver',
            name="robione_ros2_driver",
            executable = 'robione_ros2_node',
            output = 'screen',
            parameters=[config]
        )
    ])