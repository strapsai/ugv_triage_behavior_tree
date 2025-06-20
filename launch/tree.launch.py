from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the system'
    )

    # Resolve the config path for robot.tree
    config_path = os.path.join(
        get_package_share_directory('ugv_triage_behavior_tree'),
        'config',
        'toplevel.tree'
    )

    # Define the behavior tree node with namespace set directly
    behavior_tree_node = Node(
        package='behavior_tree',
        executable='behavior_tree_implementation',
        name='behavior_tree',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'config': config_path,
                'timeout': 1.0
            }
        ]
    )

    return LaunchDescription([
        namespace_arg,
        behavior_tree_node
    ])
