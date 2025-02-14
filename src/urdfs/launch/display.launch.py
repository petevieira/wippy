import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'urdfs'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'inverted_pendulum.urdf')
    print(urdf_file)
    urdf = open(urdf_file).read()
    print(urdf)

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Flag to enable joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
        )
    ])
