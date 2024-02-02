from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable
from launch.actions import GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

import os



def generate_launch_description():
    # Get the launch directory
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    pkg_simtb3_mapping = FindPackageShare('simtb3_mapping')
    pkg_simtb3_navigation = FindPackageShare('simtb3_navigation')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_localization = LaunchConfiguration('use_localization')
    rviz_file = LaunchConfiguration('rviz_file')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True)
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_nav2_cmd = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        choices=['True', 'False'],
        description='Whether to run Nav2')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_simtb3_mapping,
             'map',
             'default_map.yaml']),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_simtb3_navigation,
             'config',
             'nav2.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_rviz_file_cmd = DeclareLaunchArgument(
        'rviz_file',
        default_value=PathJoinSubstitution(
            [pkg_simtb3_navigation,
                'config',
                'map2d_nav.rviz']),
        description='Full path to the RVIZ config file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization',
        default_value='True',
        description='Whether to start robot localization')


    # Specify the actions
    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('simtb3_localization'), 'launch', 'ekf.launch.py']
                    )
                ),
                condition=IfCondition(use_localization),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 
                    'launch',
                    'bringup_launch.py'])),
            launch_arguments={
                'map': map_yaml_file,
                'autostart': autostart,
                'params_file': configured_params,
                'use_sim_time': use_sim_time
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_nav2_bringup, 
                    'launch', 
                    'rviz_launch.py'])),
            launch_arguments={
                'rviz_config': rviz_file
                }.items(),                   
            condition=IfCondition(use_rviz)
        ),

        Node(
            package='simtb3_navigation',
            executable='map2d_waypoints_follower',
            name='map2d_waypoints_follower',
            output='screen'
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_nav2_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(bringup_cmd_group)



    return ld