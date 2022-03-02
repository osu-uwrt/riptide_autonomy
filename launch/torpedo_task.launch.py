import launch
import launch.actions
from launch_ros.actions import Node

robot = "tempest"

def generate_launch_description():
    return launch.LaunchDescription([
        # create the nodes    
        Node(
            package='riptide_autonomy2',
            executable='doTask',
            name='buoy_task',
            output='screen',
            namespace=robot,
            
            arguments=["torpedo_tree.xml"]
        )
    ])