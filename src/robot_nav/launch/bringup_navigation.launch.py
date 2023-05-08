from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('simulation_enable',              default_value='false' ,description=''),
        DeclareLaunchArgument('simulation_gui',                 default_value='false',description=''),
        DeclareLaunchArgument('simulation_world',               default_value='classroom',description=''),
        DeclareLaunchArgument('gui_teleop_keyboard_enable',     default_value='true',description=''),
        DeclareLaunchArgument('gui_rviz_enable',                default_value='true',description=''),
        DeclareLaunchArgument('gui_rviz_file',                  default_value=[get_package_share_directory('robot_nav'),'/config/rviz/navigation.rviz'],description=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('robot_description'), '/launch/bringup.launch.py']),
            launch_arguments = {
                'robot_name':                   'robot',
                'simulation_enable':            LaunchConfiguration('simulation_enable'),
                'simulation_world':             LaunchConfiguration('simulation_world'),
                "simulation_gui":               LaunchConfiguration('simulation_gui'),
                "gui_teleop_keyboard_enable":   LaunchConfiguration('gui_teleop_keyboard_enable'),
                "gui_teleop_joy_enable":        LaunchConfiguration('gui_teleop_joy_enable'),
                "gui_rviz_enable":              LaunchConfiguration('gui_rviz_enable'),
                "gui_rviz_file":                LaunchConfiguration('gui_rviz_file') 
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/bringup_launch.py']),
            launch_arguments={
                'map':          [get_package_share_directory('robot_nav'),'/data/map/',LaunchConfiguration('simulation_world'),'/map.yaml'],
                'use_sim_time': LaunchConfiguration('simulation_enable'),
                'params_file':  [get_package_share_directory('robot_nav'), '/config/navigation.yaml'],
                'slam':         'False'
            }.items(),
        ),

        # Node(
        #    name='waypoints',
        #    package='robot_nav',
        #    executable='goThroughPoses',
        #    output='screen',
        #),

    ])


