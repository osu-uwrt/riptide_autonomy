import launch
import launch.actions
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # create the nodes    
        Node(
            package='riptide_autonomy',
            executable='doTask',
            name='buoy_task',
            output='screen',
            
            arguments=["search_tree.xml"]
        )
    ])