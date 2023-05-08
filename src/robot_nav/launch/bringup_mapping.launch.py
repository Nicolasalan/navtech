from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# Para salvar o mapa, execute:
# ros2 run nav2_map_server map_saver_cli 

def generate_launch_description():

    cartographer_config_dir = [get_package_share_directory('robot_nav'), '/config/']
    configuration_basename = 'cartographer.lua'

    return LaunchDescription([
     
        DeclareLaunchArgument('simulation_enable',              default_value='false' ,description=''),
        DeclareLaunchArgument('simulation_gui',                 default_value='false',description=''),
        DeclareLaunchArgument('simulation_world',               default_value='classroom',description=''),
        DeclareLaunchArgument('gui_teleop_keyboard_enable',     default_value='true',description=''),
        DeclareLaunchArgument('gui_rviz_enable',                default_value='true',description=''),
        DeclareLaunchArgument('gui_rviz_file',                  default_value=[get_package_share_directory('robot_nav'),'/config/rviz/mapping.rviz'],description=''),
        
        DeclareLaunchArgument('resolution',                     default_value='0.05', description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument('publish_period_sec',             default_value='1.0', description='OccupancyGrid publishing period'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('robot_description'), '/launch/bringup.launch.py']),
            launch_arguments = {
                'robot_name':                   'robot',
                'simulation_enable':            LaunchConfiguration('simulation_enable'),
                'simulation_world':             LaunchConfiguration('simulation_world'),
                "simulation_gui":               LaunchConfiguration('simulation_gui'),
                "gui_teleop_keyboard_enable":   LaunchConfiguration('gui_teleop_keyboard_enable'),
                "gui_rviz_enable":              LaunchConfiguration('gui_rviz_enable'),
                "gui_rviz_file":                LaunchConfiguration('gui_rviz_file') 
            }.items()
        ),


        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            remappings=[
                ('scan', 'base_scan_front_filtered'),
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('simulation_enable')
            }],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('simulation_enable')
            }],
            arguments=['-resolution', LaunchConfiguration('resolution'), '-publish_period_sec', LaunchConfiguration('publish_period_sec')]
        ),

    ])
