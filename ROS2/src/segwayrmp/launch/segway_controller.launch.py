import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='segway_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='segwayrmp',
                    plugin='robot::Chassis',
                    name='segway_chassis',
                    remappings=[("/cmd_vel", "/out_cmd_vel")],
                    parameters=[{
                        'comu_interface': 'can'
                    }]
                ),
                ComposableNode(
                    package='segwayrmp',
                    plugin='segway::SegwayController',
                    name='segway_controller',
                    remappings=[('/in_cmd_vel', '/twist_mux/cmd_vel')]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
