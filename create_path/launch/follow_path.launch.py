from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to the map file for navigation'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Enable simulation mode'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable RViz visualization'
    )

    cycles_arg = DeclareLaunchArgument(
        'cycles',
        default_value='1',
        description='Number of navigation cycles to perform'
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('linorobot2_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'sim': LaunchConfiguration('sim'),
            'rviz': LaunchConfiguration('rviz'),
            'map': LaunchConfiguration('map')
        }.items()
    )


    node = Node(
        package='create_path',
        executable='follow_path',
        name='follow_path',
        arguments=[
            LaunchConfiguration('cycles'),
        ],
        output='screen'
    )

    node2 = Node(
        package='create_path',
        executable='show_ocgm',
        name='show_ocgm',
        output='screen'
    )

    return LaunchDescription([
        map_path_arg,
        sim_arg,
        rviz_arg,
        navigation_launch,
        node,node2,
        cycles_arg 
    ])