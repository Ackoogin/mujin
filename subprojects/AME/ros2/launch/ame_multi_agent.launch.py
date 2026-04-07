from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    """Launch multi-agent AME deployment.
    
    Spawns:
    - 1 WorldModelNode (shared state)
    - 1 PlannerNode (shared planner)
    - N ExecutorNodes (one per agent, namespaced by agent_id)
    - 1 LifecycleManager
    
    Each ExecutorNode publishes/subscribes to:
    - /{agent_id}/executor/bt_xml
    - /{agent_id}/executor/bt_events
    - /{agent_id}/executor/status
    """
    
    # Default agent IDs - can be overridden via launch argument
    default_agents = 'uav1,uav2'
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'pddl_file',
            default_value='',
            description='Path to PDDL domain file'),
        DeclareLaunchArgument(
            'problem_file',
            default_value='',
            description='Path to PDDL problem file'),
        DeclareLaunchArgument(
            'agents',
            default_value=default_agents,
            description='Comma-separated list of agent IDs'),

        # Shared WorldModelNode
        LifecycleNode(
            package='ame_ros2',
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

        # Shared PlannerNode
        LifecycleNode(
            package='ame_ros2',
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

        # Agent 1 ExecutorNode
        LifecycleNode(
            package='ame_ros2',
            executable='executor_node',
            name='executor_node_uav1',
            output='screen',
            parameters=[{
                'agent_id':        'uav1',
                'tick_rate_hz':    50.0,
                'bt_log.enabled':  True,
                'bt_log.path':     'bt_events_uav1.jsonl',
                'world_model_node': 'world_model_node',
            }],
        ),

        # Agent 2 ExecutorNode
        LifecycleNode(
            package='ame_ros2',
            executable='executor_node',
            name='executor_node_uav2',
            output='screen',
            parameters=[{
                'agent_id':        'uav2',
                'tick_rate_hz':    50.0,
                'bt_log.enabled':  True,
                'bt_log.path':     'bt_events_uav2.jsonl',
                'world_model_node': 'world_model_node',
            }],
        ),

        # Agent dispatcher for coordinating multi-agent goal dispatch
        LifecycleNode(
            package='ame_ros2',
            executable='agent_dispatcher_node',
            name='agent_dispatcher_node',
            output='screen',
            parameters=[{
                'world_model_node': 'world_model_node',
            }],
        ),

        # Lifecycle manager for all nodes
        Node(
            package='ame_ros2',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'managed_nodes': [
                    'world_model_node',
                    'planner_node',
                    'agent_dispatcher_node',
                    'executor_node_uav1',
                    'executor_node_uav2',
                ],
                'transition_timeout_ms': 5000,
            }],
        ),
    ])
