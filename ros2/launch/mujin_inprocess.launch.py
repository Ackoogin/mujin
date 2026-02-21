from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

        Node(
            package='mujin_ros2',
            executable='mujin_combined',
            name='mujin_combined',
            output='screen',
            parameters=[{
                'domain.pddl_file':     LaunchConfiguration('pddl_file'),
                'domain.problem_file':  LaunchConfiguration('problem_file'),
                'tick_rate_hz':         50.0,
                'publish_rate_hz':      10.0,
                'compiler.parallel':    False,
                'audit_log.enabled':    True,
                'bt_log.enabled':       True,
            }],
        ),
    ])
