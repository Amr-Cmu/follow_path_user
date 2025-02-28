from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
 
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Enable simulation mode'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Enable RViz visualization'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('linorobot2_navigation'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'sim': LaunchConfiguration('sim'),
            'rviz': LaunchConfiguration('rviz')
        }.items()
    )

    save_path_node = Node(
        package='create_path',
        executable='save_path',
        name='save_path',
    )

    # Create and return launch description
    return LaunchDescription([
        sim_arg,
        rviz_arg,
        slam_launch,
        save_path_node
    ])


