import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(get_package_share_directory('robot'), 'rviz', 'tf.rviz'))

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a joint_state_publisher node
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # Create Rviz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_file]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz
        
    ])
