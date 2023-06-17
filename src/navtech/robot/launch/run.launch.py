import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():


     # ====================== ROBOT ====================== #
     # Check if we're told to use sim time
     use_sim_time = LaunchConfiguration('use_sim_time', default='True')
     rviz_file = LaunchConfiguration('rviz_file', default=os.path.join(get_package_share_directory('robot'), 'rviz', 'slam.rviz'))

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

     # ====================== Gazebo ====================== #

     package_name='robot'

     world = os.path.join(get_package_share_directory(package_name),'worlds','simulation.world')

     # Include the Gazebo launch file, provided by the gazebo_ros package
     gazebo = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
     )

     # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
     spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                         arguments=['-topic', 'robot_description',
                                        '-entity', 'robot'],
                         output='screen'
     )

     # ====================== SLAM ====================== #

     use_sim_time = LaunchConfiguration('use_sim_time')
     params_file = LaunchConfiguration('params_file')
     default_params_file = os.path.join(get_package_share_directory("robot"),
                                        'config', 'params_async.yaml')

     declare_use_sim_time_argument = DeclareLaunchArgument(
          'use_sim_time',
          default_value='true',
          description='Use simulation/Gazebo clock')
     declare_params_file_cmd = DeclareLaunchArgument(
          'params_file',
          default_value=default_params_file,
          description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

     has_node_params = HasNodeParams(source_file=params_file,
                                        node_name='slam_toolbox')

     actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                             ' else "', default_params_file, '"'])

     log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                        ' does not contain slam_toolbox parameters. Using default: ',
                                        default_params_file],
                                   condition=UnlessCondition(has_node_params))

     start_async_slam_toolbox_node = Node(
          parameters=[
               actual_params_file,
               {'use_sim_time': use_sim_time}
          ],
          package='slam_toolbox',
          executable='async_slam_toolbox_node',
          name='slam_toolbox',
          output='screen')

     ld = LaunchDescription()

     ld.add_action(declare_use_sim_time_argument)
     ld.add_action(declare_params_file_cmd)
     ld.add_action(log_param_change)
     ld.add_action(start_async_slam_toolbox_node)

     # ====================== Navigation ====================== #

     # Get the launch directory
     bringup_dir = get_package_share_directory('robot')

     # parameters to pass to the nav2 stack
     namespace = LaunchConfiguration('namespace')
     use_sim_time = LaunchConfiguration('use_sim_time')
     autostart = LaunchConfiguration('autostart')
     params_file = LaunchConfiguration('params_file')
     default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
     map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

     # nodes to launch
     lifecycle_nodes = ['controller_server',
                         'planner_server',
                         'recoveries_server',
                         'bt_navigator',
                         'waypoint_follower']

     remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]

     # Create our own temporary YAML files that include substitutions
     param_substitutions = {
          'use_sim_time': use_sim_time,
          'default_bt_xml_filename': default_bt_xml_filename,
          'autostart': autostart,
          'map_subscribe_transient_local': map_subscribe_transient_local}

     configured_params = RewrittenYaml(
               source_file=params_file,
               root_key=namespace,
               param_rewrites=param_substitutions,
               convert_types=True)

     # Launch!
     return LaunchDescription([
          DeclareLaunchArgument(
               'use_sim_time',
               default_value='false',
               description='Use sim time if true'),

          
          node_robot_state_publisher,
          node_joint_state_publisher,
          node_rviz,
          gazebo,
          spawn_entity,
          ld
        
     ])
