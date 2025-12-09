from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='test_world',
        description='Name of the map to use for testing'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='test_world.sdf',
        description='SDF world file for testing'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot model'
    )

    validate_arg = DeclareLaunchArgument(
        'validate',
        default_value='true',
        description='Whether to run validation after setup'
    )

    # Get launch configurations
    map_name = LaunchConfiguration('map')
    world_name = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    validate = LaunchConfiguration('validate')

    # Launch Gazebo with test world
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('physical_ai_examples'),
                'simulation',
                world_name
            ])
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Spawn robot in the world
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # Launch Nav2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': True}.items()
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'slam': 'False',
            'map': PathJoinSubstitution([
                FindPackageShare('physical_ai_examples'),
                'maps',
                [map_name, '.yaml']
            ]),
            'use_sim_time': True,
            'params_file': PathJoinSubstitution([
                FindPackageShare('physical_ai_examples'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    # Launch RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('physical_ai_examples'),
                'rviz',
                'navigation_validation.rviz'
            ])
        ],
        condition=IfCondition(validate)
    )

    # Launch navigation validator after system is ready (with delay)
    navigation_validator_node = TimerAction(
        period=10.0,  # Wait 10 seconds for system to initialize
        actions=[
            Node(
                package='physical_ai_examples',
                executable='navigation_validator.py',
                name='navigation_validator',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(validate)
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        world_arg,
        robot_name_arg,
        validate_arg,
        gzserver_launch,
        gzclient_launch,
        spawn_robot,
        robot_state_publisher,
        nav2_launch,
        bringup_launch,
        rviz_node,
        navigation_validator_node
    ])