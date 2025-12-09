from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    benchmark_duration_arg = DeclareLaunchArgument(
        'benchmark_duration',
        default_value='30',
        description='Duration of benchmark in seconds'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Rate at which to publish test data (Hz)'
    )

    # Get launch configurations
    benchmark_duration = LaunchConfiguration('benchmark_duration')
    publish_rate = LaunchConfiguration('publish_rate')

    # Perception benchmark node
    perception_benchmark_node = Node(
        package='physical_ai_examples',
        executable='perception_benchmark.py',
        name='perception_benchmark',
        parameters=[
            {'benchmark_duration': benchmark_duration},
            {'publish_rate': publish_rate}
        ],
        output='screen'
    )

    return LaunchDescription([
        benchmark_duration_arg,
        publish_rate_arg,
        perception_benchmark_node
    ])