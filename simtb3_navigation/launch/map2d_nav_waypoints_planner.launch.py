from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
import os

def generate_launch_description():

    pkg_simtb3_navigation = FindPackageShare('simtb3_navigation')
    pkg_simtb3_mapping = FindPackageShare('simtb3_mapping')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    # Create the launch configuration variables

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution(
            [pkg_simtb3_navigation, 
            'config', 
            'map2d_nav.rviz']),
        description='Full path to the RVIZ config file to use')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_simtb3_mapping, 'map', 'default_map.yaml']),
        description='Full path to map yaml file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    # run map server
    map_server_group_action = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[            
                {'yaml_filename': map_yaml_file,
                 'use_sim_time': use_sim_time,
                 'frame_id': 'map',
                 'topic_name': 'map'},
            ],
            remappings=remappings),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'autostart': autostart},
                        {'node_names': ['map_server']}])
    ])
    
    # run rviz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_nav2_bringup, 
                'launch', 
                'rviz_launch.py'])),
        launch_arguments={
            'rviz_config': rviz_config_file,
            }.items()           
    )

    # run interactive pose logger
    map2d_waypoints_planner_cmd = Node(
        package='simtb3_navigation',
        executable='map2d_waypoints_planner',
        name='map2d_waypoints_planner',
        output='screen',
    )
    

    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(map_server_group_action)
    ld.add_action(map2d_waypoints_planner_cmd)
    ld.add_action(rviz_cmd)

    return ld