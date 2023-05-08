from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='slinky',description=''),

        Node(
            name='robot_base_odom',
            package='robot_description',
            executable='odom',
            output='screen',
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidarNode_front',
            remappings=[
                ('scan', 'base_scan_front')
            ],
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/laser_front_rplidar_s2', 
                         'serial_baudrate': 1000000, 
                         'frame_id': 'base_scan_front',
                         'inverted': True, 
                         'angle_compensate': True,
                         'scan_mode': 'Standard'
                         }],
            output='screen'),

    ])

    
