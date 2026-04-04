from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    """Launch multi-planner AME deployment.

    Spawns:
    - 1 WorldModelNode (shared state)
    - N PlannerNodes (each with its own domain, loaded via ~/load_domain service)
    - N ExecutorNodes (one per agent, namespaced by agent_id)
    - 1 LifecycleManager

    Domain models are loaded at runtime via each planner's ~/load_domain service
    rather than from file paths. This supports backend-driven domain loading where
    the devenv pushes domain PDDL via service calls.

    Each PlannerNode exposes:
    - ~/<node_name>/plan          (action server)
    - ~/<node_name>/load_domain   (service, available after configure)
    - ~/<node_name>/bt_xml        (publisher)

    Usage:
      ros2 launch ame_ros2 ame_multi_planner.launch.py

    Then load domains via service calls:
      ros2 service call /planner_logistics/load_domain ame_ros2/srv/LoadDomain \
        "{domain_id: 'logistics', domain_pddl: '...', problem_pddl: '...'}"
      ros2 service call /planner_surveillance/load_domain ame_ros2/srv/LoadDomain \
        "{domain_id: 'surveillance', domain_pddl: '...', problem_pddl: '...'}"
    """

    return LaunchDescription([
        # Shared WorldModelNode
        LifecycleNode(
            package='ame_ros2',
            executable='world_model_node',
            name='world_model_node',
            output='screen',
            parameters=[{
                'audit_log.enabled':   True,
                'audit_log.path':      'wm_audit.jsonl',
                'publish_rate_hz':     10.0,
            }],
        ),

        # Planner 1: logistics domain
        LifecycleNode(
            package='ame_ros2',
            executable='planner_node',
            name='planner_logistics',
            output='screen',
            parameters=[{
                'world_model_node':    'world_model_node',
                'plan_audit.enabled':  True,
                'plan_audit.path':     'plan_audit_logistics.jsonl',
                'compiler.parallel':   False,
            }],
        ),

        # Planner 2: surveillance domain
        LifecycleNode(
            package='ame_ros2',
            executable='planner_node',
            name='planner_surveillance',
            output='screen',
            parameters=[{
                'world_model_node':    'world_model_node',
                'plan_audit.enabled':  True,
                'plan_audit.path':     'plan_audit_surveillance.jsonl',
                'compiler.parallel':   False,
            }],
        ),

        # Executor for logistics agent
        LifecycleNode(
            package='ame_ros2',
            executable='executor_node',
            name='executor_node_logistics',
            output='screen',
            parameters=[{
                'agent_id':         'logistics_agent',
                'tick_rate_hz':     50.0,
                'bt_log.enabled':   True,
                'bt_log.path':      'bt_events_logistics.jsonl',
                'world_model_node': 'world_model_node',
            }],
        ),

        # Executor for surveillance agent
        LifecycleNode(
            package='ame_ros2',
            executable='executor_node',
            name='executor_node_surveillance',
            output='screen',
            parameters=[{
                'agent_id':         'surveillance_agent',
                'tick_rate_hz':     50.0,
                'bt_log.enabled':   True,
                'bt_log.path':      'bt_events_surveillance.jsonl',
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
                    'planner_logistics',
                    'planner_surveillance',
                    'executor_node_logistics',
                    'executor_node_surveillance',
                ],
                'transition_timeout_ms': 5000,
            }],
        ),
    ])
