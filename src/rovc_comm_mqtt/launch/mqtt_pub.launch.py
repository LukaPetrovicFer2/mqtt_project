from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rovc_comm_mqtt',
            executable='pub_node',
            name='mqtt_bridge_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('rovc_comm_mqtt'),
                'config',
                'params.yaml'
            )]
        )
    ])

# Needed for get_package_share_directory
from ament_index_python.packages import get_package_share_directory

