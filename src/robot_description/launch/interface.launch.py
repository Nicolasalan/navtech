from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',                   default_value='true',description=''),
        DeclareLaunchArgument('gui_teleop_keyboard_enable',     default_value='true',description=''),
        DeclareLaunchArgument('gui_rviz_enable',                default_value='true',description=''),
        DeclareLaunchArgument('gui_rviz_file',                  default_value=[get_package_share_directory('robot_description'),'/config/rviz/robot.rviz'],description=''),

        Node(
            condition=IfCondition(LaunchConfiguration('gui_teleop_keyboard_enable')),
            name='teleop_twist_keyboard',
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix=["xterm -hold -e"]
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('gui_rviz_enable')),
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            output='own_log',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d',LaunchConfiguration('gui_rviz_file')]
        ),

    ])
    
