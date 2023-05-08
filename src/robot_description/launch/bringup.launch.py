from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('x',                              default_value='0'    ,description=''),
        DeclareLaunchArgument('y',                              default_value='0'    ,description=''),
        DeclareLaunchArgument('yaw',                            default_value='0'    ,description=''),

        DeclareLaunchArgument('simulation_world',               default_value='classroom',description=''),
        DeclareLaunchArgument('simulation_enable',              default_value='false' ,description=''),
        DeclareLaunchArgument('simulation_gui',                 default_value='false' ,description=''),
        
        DeclareLaunchArgument('gui_teleop_keyboard_enable',     default_value='false' ,description=''),
        DeclareLaunchArgument('gui_rviz_enable',                default_value='false' ,description=''),
        DeclareLaunchArgument('gui_rviz_file',                  default_value='$(find-pkg-share robot_description)/config/rviz/robot.rviz' ,description=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('robot_description'), '/launch/simulation.launch.py']),
            condition=IfCondition(LaunchConfiguration('simulation_enable')),
            launch_arguments = {
                'simulation_world':   LaunchConfiguration('simulation_world'),
                'simulation_gui':     LaunchConfiguration('simulation_gui')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('robot_description'), '/launch/devices.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('simulation_enable')),
            launch_arguments = {
                'robot_name':   LaunchConfiguration('robot_name')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('robot_description'), '/launch/load_robot.launch.py']),
            launch_arguments = {
                'use_sim_time': LaunchConfiguration('simulation_enable'),
                'robot_name':   'robot',
                'x':            LaunchConfiguration('x'),
                'y':            LaunchConfiguration('y'),
                'yaw':          LaunchConfiguration('yaw')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('robot_description'), '/launch/interface.launch.py']),
            launch_arguments = {
                'use_sim_time':                 LaunchConfiguration('simulation_enable'),
                'gui_teleop_keyboard_enable':   LaunchConfiguration('gui_teleop_keyboard_enable'),
                'gui_rviz_enable':              LaunchConfiguration('gui_rviz_enable'),
                'gui_rviz_file':                LaunchConfiguration('gui_rviz_file')
            }.items()
        ),

    ])
    