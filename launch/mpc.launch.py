from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('mpc_pkg')
    config = os.path.join(pkg_dir, 'config', 'mpc_params.yaml')

    return LaunchDescription([
        Node(
            package='mpc_pkg',
            executable='mpc_node',
            name='mpc_node',
            parameters=[config],
            output='screen',
        ),
    ])
