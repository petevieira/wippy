import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('urdfs'),
        'rviz',
        'urdf.rviz'
    )
    package_name = 'urdfs'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'wippy.urdf')
    print(urdf_file)
    urdf = open(urdf_file).read()
    print(urdf)

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Flag to enable joint_state_publisher_gui'
        ),
        # Launch rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
        # Launch robot_state_publisher for the URDF so that the robot model can be visualized in rviz2
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        ),
        # Launch joint_state_publisher to publish joint states for the robot model
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
        ),
        # Launch joint_state_publisher_gui to publish joint states for the robot
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
        )
    ])
