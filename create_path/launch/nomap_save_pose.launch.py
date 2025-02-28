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

   
    save_pose_node = Node(
        package='create_path',
        executable='create_path',
        name='create_path',
        output='screen'
    )

  
    return LaunchDescription([
        sim_arg,
        rviz_arg,
        slam_launch,
        save_pose_node
    ])
