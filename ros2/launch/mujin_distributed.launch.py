from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pddl_file',
            default_value='',
            description='Path to PDDL domain file'),
        DeclareLaunchArgument(
            'problem_file',
            default_value='',
            description='Path to PDDL problem file'),

        LifecycleNode(
            package='mujin_ros2',
            executable='world_model_node',
            name='world_model_node',
            output='screen',
            parameters=[{
                'domain.pddl_file':    LaunchConfiguration('pddl_file'),
                'domain.problem_file': LaunchConfiguration('problem_file'),
                'audit_log.enabled':   True,
                'audit_log.path':      'wm_audit.jsonl',
                'publish_rate_hz':     10.0,
            }],
        ),

        LifecycleNode(
            package='mujin_ros2',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[{
                'domain.pddl_file':    LaunchConfiguration('pddl_file'),
                'domain.problem_file': LaunchConfiguration('problem_file'),
                'world_model_node':    'world_model_node',
                'plan_audit.enabled':  True,
                'plan_audit.path':     'plan_audit.jsonl',
                'compiler.parallel':   False,
            }],
        ),

        LifecycleNode(
            package='mujin_ros2',
            executable='executor_node',
            name='executor_node',
            output='screen',
            parameters=[{
                'tick_rate_hz':    50.0,
                'bt_log.enabled':  True,
                'bt_log.path':     'bt_events.jsonl',
                'world_model_node': 'world_model_node',
            }],
        ),

        Node(
            package='mujin_ros2',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'managed_nodes': [
                    'world_model_node',
                    'planner_node',
                    'executor_node',
                ],
                'transition_timeout_ms': 5000,
            }],
        ),
    ])
