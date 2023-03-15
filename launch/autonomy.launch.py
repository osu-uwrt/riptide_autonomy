import launch
import launch.actions
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('robot', default_value="tempest", description="Name of the vehicle"),
        launch.actions.GroupAction(
            actions=[
                PushRosNamespace(
                    LC("robot"),
                ),

                # create the nodes
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='torpedo_hole_publisher',
                    arguments=["0", "0.1524", "0.381", "0", "0", "0", "torpedoGman_frame", "torpedoHole_frame"]
                ),

                Node(
                    package='riptide_autonomy2',
                    executable='doTask',
                    name='autonomy',
                    respawn=True,
                    output='screen',
                    parameters=[
                        {
                            # 'ext_plugin_list': [
                                
                            # ],
                            'enable_zmq': False,
                            'enable_cout': False
                        }
                    ]
                )
            ], scoped=True
        )
    ])

