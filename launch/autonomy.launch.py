import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        # create the nodes
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='torpedo_hole_publisher',
            arguments=['0', '0.1524', '0.381', '0' '0', '0', 'torpedoGman_frame', 'torpedoHole_frame']
        ),
         
        launch_ros.actions.Node(
            package='riptide_autonomy2',
            executable='AlignTorpedos.py',
            name='AlignTorpedos',
            respawn=False,
            output='screen',
        )
    ])