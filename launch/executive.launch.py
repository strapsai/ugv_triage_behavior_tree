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

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Prefix for the topics. Should have trailing and leading backslash (e.g., /ugv/ to match the topic names in the tree.'
    )

    init_timeout_ms_arg = DeclareLaunchArgument(
        'init_timeout_ms',
        default_value='5000',
        description='Timeout for initialization'
    )

    system_init_arg = DeclareLaunchArgument(
        'init_topic',
        default_value=[LaunchConfiguration("prefix"), 'behavior/top_level_tree/init_sub'],
        description='Topic to listen to which indicates system is ready'
    )

    # State of the system is defined as UInt8 where
    # 0 = Idle
    # 1 = Explore
    # 2 = Inspect
    current_state_arg = DeclareLaunchArgument(
        'current_state_topic',
        default_value='behavior/top_level_tree/current_state_int',
        description='Current state of the system in the form of UInt8'
    )
    state_selection_topic_arg = DeclareLaunchArgument(
        'state_selection_topic',
        default_value='behavior/top_level_tree/select_state',
        description='Current state of the system in the form of UInt8.'
    )

    in_explore_mode_arg = DeclareLaunchArgument(
        'explore_mode_topic',
        default_value='behavior/top_level_tree/in_explore_mode',
        description='True if in explore mode'
    )

    in_inspect_mode_arg = DeclareLaunchArgument(
        'inspect_mode_topic',
        default_value='behavior/top_level_tree/in_inspect_mode',
        description='True if in inspect mode'
    )

    # Define the behavior tree node with namespace set directly
    behavior_tree_node = Node(
        package='ugv_top_behavior_tree',
        executable='ugv_top_behavior_tree_node',
        name='behavior_executive',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'init_timeout_ms': LaunchConfiguration('init_timeout_ms'),
                'init_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('init_topic')],
                'current_state_topic': [LaunchConfiguration('prefix'),LaunchConfiguration('current_state_topic')],
                'explore_mode_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('explore_mode_topic')],
                'inspect_mode_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('inspect_mode_topic')],
                'state_selection_topic': [LaunchConfiguration('prefix'),LaunchConfiguration('state_selection_topic')]
            }
        ]
    )

    return LaunchDescription([
        namespace_arg,
        prefix_arg,
        init_timeout_ms_arg,
        system_init_arg,
        current_state_arg,
        state_selection_topic_arg,
        in_explore_mode_arg,
        in_inspect_mode_arg,
        behavior_tree_node
    ])
