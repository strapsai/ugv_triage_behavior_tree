from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        default_value=[LaunchConfiguration("prefix"), 'behavior/init_sub'],
        description='Topic to listen to which indicates system is ready'
    )

    estop_topic_arg = DeclareLaunchArgument(
        'estop_topic',
        default_value=[LaunchConfiguration("prefix"), 'behavior/estop_sub'],
        description='Topic to listen for estop signal'
    )

    # State of the system is defined as UInt8 where
    # 0 = Idle
    # 1 = Explore
    # 2 = Inspect
    current_state_arg = DeclareLaunchArgument(
        'current_state_topic',
        default_value='behavior/current_state_int',
        description='Current state of the system in the form of UInt8'
    )

    state_selection_topic_arg = DeclareLaunchArgument(
        'state_selection_topic',
        default_value='behavior/select_state',
        description='Current state of the system in the form of UInt8.'
    )

    observed_casualty_topic_arg = DeclareLaunchArgument(
        'observed_casualty_topic',
        default_value='observed_casualty_topic', #humanflow::msg::ReIDPersonArray
        description='List of identified casualties'
    )

    robot_gps_topic_arg = DeclareLaunchArgument(
        'robot_gps_topic',
        default_value='robot_gps_topic', #sensor_msgs::msg::NavSatFix
        description='GPS position of the Robot as NavSatFix'
    )

#    in_explore_mode_arg = DeclareLaunchArgument(
#        'explore_mode_topic',
#        default_value='behavior/in_explore_mode',
#        description='True if in explore mode'
#    )
#
#    in_inspect_mode_arg = DeclareLaunchArgument(
#        'inspect_mode_topic',
#        default_value='behavior/in_inspect_mode',
#        description='True if in inspect mode'
#    )


    milestone_in_topic_arg = DeclareLaunchArgument(
        'milestone_in_topic',
        default_value='plan_executor/milestone_announce',
        description='Topic on which to expect reached milestones'
    )

    milestone_out_topic_arg = DeclareLaunchArgument(
        'milestone_out_topic',
        default_value='plan_executor/milestone_clearance',
        description='Topic on which to clear reached milestones'
    )

    inspection_plan_request_topic_arg = DeclareLaunchArgument(
        'inspection_plan_request_topic',
        default_value='plan_executor/plan_request', #for Kabir
        description='Topic on which to clear reached milestones'
    )

    exploration_request_topic_arg = DeclareLaunchArgument(
        'exploration_request_topic',
        default_value='/tbd2', #for Kabir
        description='Topic for requesting exploration activities'
    )

    rd_request_topic_arg = DeclareLaunchArgument(
        'rd_request_topic',
        default_value='algorithm/rd_request', #for Kabir
        description='Topic on which to clear reached milestones'
    )
   
    hemo_request_topic_arg = DeclareLaunchArgument(
        'hemo_request_topic',
        default_value='algorithm/hemo_request', #for Kabir
        description='Topic on which to clear reached milestones'
    )
   
    rd_nodestate_topic_arg = DeclareLaunchArgument(
        'rd_nodestate_topic',
        default_value='algorithm/rd_nodestate', #for Kabir
        description='Topic on which to clear reached milestones'
    )
   
    hemo_nodestate_topic_arg = DeclareLaunchArgument(
        'hemo_nodestate_topic',
        default_value='algorithm/hemo_nodestate', #for Kabir
        description='Topic on which to clear reached milestones'
    )

    manual_mode_topic_arg = DeclareLaunchArgument(
        'manual_mode_topic',
        default_value='behavior/in_manual_mode',
        description='Topic declaring activation of the manual mode'
    )


    # Define the behavior tree node with namespace set directly
    behavior_tree_node = Node(
        package='ugv_triage_behavior_tree',
        executable='ugv_triage_behavior_tree_node',
        name='behavior_executive',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'init_timeout_ms': LaunchConfiguration('init_timeout_ms'),
                'init_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('init_topic')],
                'estop_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('estop_topic')],
                'current_state_topic': [LaunchConfiguration('prefix'),LaunchConfiguration('current_state_topic')],
                #'explore_mode_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('explore_mode_topic')],
                #'inspect_mode_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('inspect_mode_topic')],
                'state_selection_topic': [LaunchConfiguration('prefix'),LaunchConfiguration('state_selection_topic')],
                'manual_mode_topic' :  [LaunchConfiguration('prefix'),LaunchConfiguration('manual_mode_topic')],

                'observed_casualty_topic': [LaunchConfiguration("prefix"),LaunchConfiguration('observed_casualty_topic')],
                'robot_gps_topic': [LaunchConfiguration("prefix"),LaunchConfiguration('robot_gps_topic')],
                'exploration_request_topic': [LaunchConfiguration("prefix"),LaunchConfiguration('exploration_request_topic')],

                'milestone_in_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('milestone_in_topic')],
                'milestone_out_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('milestone_out_topic')],
                'plan_request_topic': [LaunchConfiguration('prefix'), LaunchConfiguration('inspection_plan_request_topic')],
                'rd_request_topic' : [LaunchConfiguration('prefix'), LaunchConfiguration('rd_request_topic')],
                'hemo_request_topic' : [LaunchConfiguration('prefix'), LaunchConfiguration('hemo_request_topic')],
                'rd_nodestate_topic' : [LaunchConfiguration('prefix'), LaunchConfiguration('rd_nodestate_topic')],
                'hemo_nodestate_topic' : [LaunchConfiguration('prefix'), LaunchConfiguration('hemo_nodestate_topic')]
            }
        ]
    )


    behavior_tree_launch_file = get_package_share_directory('ugv_triage_behavior_tree') + '/launch/tree.launch.py'
    tree_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(behavior_tree_launch_file),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'prefix': LaunchConfiguration('prefix')
            }.items(),
        )



#    exploration_launch_file = get_package_share_directory('ugv_exploration_behavior_tree') + '/launch/executive.launch.py'
#    exploration_launcher = IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(exploration_launch_file),
#            launch_arguments={
#                'namespace': LaunchConfiguration('namespace'),
#                'prefix': LaunchConfiguration('prefix'),
#                'observed_casualty_topic': LaunchConfiguration('observed_casualty_topic'),
#                'robot_gps_topic': LaunchConfiguration('robot_gps_topic'),
#                'exploration_request_topic': LaunchConfiguration('exploration_request_topic'),
#                'state_selection_topic': LaunchConfiguration('state_selection_topic')
#            }.items(),
#        )
#
#    inspection_launch_file = get_package_share_directory('ugv_inspection_behavior_tree') + '/launch/executive.launch.py'
#    inspection_launcher = IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(inspection_launch_file),
#            launch_arguments={
#                'namespace': LaunchConfiguration('namespace'),
#                'prefix': LaunchConfiguration('prefix'),
#                'milestone_in_topic':  LaunchConfiguration('milestone_in_topic'),
#                'milestone_out_topic':  LaunchConfiguration('milestone_out_topic'),
#                'inspection_plan_request_topic':  LaunchConfiguration('inspection_plan_request_topic'),
#                'rd_request_topic' :  LaunchConfiguration('rd_request_topic'),
#                'hemo_request_topic' :  LaunchConfiguration('hemo_request_topic'),
#                'rd_nodestate_topic' :  LaunchConfiguration('rd_nodestate_topic'),
#                'hemo_nodestate_topic' :  LaunchConfiguration('hemo_nodestate_topic')
#            }.items(),
#        )


    return LaunchDescription([
        namespace_arg,
        prefix_arg,
        init_timeout_ms_arg,
        system_init_arg,
        estop_topic_arg,
        current_state_arg,
        state_selection_topic_arg,
        manual_mode_topic_arg,

        observed_casualty_topic_arg,
        robot_gps_topic_arg,
        exploration_request_topic_arg,

        milestone_in_topic_arg,
        milestone_out_topic_arg,
        inspection_plan_request_topic_arg,
        rd_request_topic_arg,
        hemo_request_topic_arg,
        rd_nodestate_topic_arg,
        hemo_nodestate_topic_arg,

        behavior_tree_node,

        tree_launcher,
    ])
