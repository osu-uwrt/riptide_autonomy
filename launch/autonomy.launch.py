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

                Node(
                    package='riptide_autonomy2',
                    executable='doTask',
                    name='autonomy',
                    respawn=False,
                    output='screen',
                    parameters=[
                        {
                            'enable_zmq': False,
                            'enable_cout': False
                        }
                    ]
                ),
                
                # Node(
                #     package='riptide_autonomy2',
                #     executable='HeadlessInterface.py',
                #     name='headless_interface',
                #     output='screen',
                #     parameters=[
                #         {
                #             # behaviortree file specified relative to the "trees" directory in install
                #             'behaviortree_to_run': "TorpedoTree.xml"
                #         }
                #     ]
                # )
            ], scoped=True
        )
    ])

