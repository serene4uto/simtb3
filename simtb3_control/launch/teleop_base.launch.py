from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('simtb3_control'), 'config', 'twist_mux.yaml']
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/cmd_vel')},
        parameters=[filepath_config_twist_mux]
    )

    ld = LaunchDescription()
    # ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_twist_mux)
    return ld
