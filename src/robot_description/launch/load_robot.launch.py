import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.descriptions import ParameterValue 
from launch.substitutions import Command

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',description=''),
        DeclareLaunchArgument('x',            default_value='0',description=''),
        DeclareLaunchArgument('y',            default_value='0',description=''),
        DeclareLaunchArgument('yaw',          default_value='0',description=''),

        Node(
            name='laser_filter',
            package='robot_description',
            executable='laser_filter',
            output='screen'
        ),    

        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'), 
                'robot_description': ParameterValue(Command(['cat ', get_package_share_directory('robot_description'),'/config/robot/','robot.urdf']), value_type=str),
            }],
        ),

        Node(
            name='joint_state_publisher',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),    

    ])

