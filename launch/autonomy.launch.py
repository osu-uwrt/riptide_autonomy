import launch
import launch.actions
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.GroupAction(
            actions=[
                PushRosNamespace("/tempest"),
                # create the nodes
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='torpedo_hole_publisher',
                    arguments=["0", "0.1524", "0.381", "0", "0", "0", "torpedoGman_frame", "torpedoHole_frame"]
                ),
                
                Node(
                    package='riptide_autonomy2',
                    executable='AlignTorpedos.py',
                    name='AlignTorpedos',
                    respawn=False,
                    output='screen',
                )
            ]
        )
    ])

