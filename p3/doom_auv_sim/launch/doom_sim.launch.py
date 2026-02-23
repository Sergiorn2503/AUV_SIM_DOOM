import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'doom_auv_sim'
    pkg_share = get_package_share_directory(pkg_name)
    
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'doom_sim.rviz')
    
    return LaunchDescription([
        # Main Simulation Node
        Node(
            package=pkg_name,
            executable='doom_sim_node',
            name='doom_sim_node',
            output='screen'
        ),
        
        
        # Acoustic Channel Node
        Node(
            package=pkg_name,
            executable='acoustic_channel_node',
            name='acoustic_channel_node',
            output='screen'
        ),

        # Visualization Bridge Node (Realistic Lag)
        Node(
             package=pkg_name,
             executable='viz_bridge_node',
             name='viz_bridge_node',
             output='screen'
        ),

        # Command Arbitrator Node (Central Control Logic)
        Node(
            package=pkg_name,
            executable='arbitrator_node',
            name='arbitrator_node',
            output='screen'
        ),

        # Mission Node (Autonomous Logic)
        Node(
            package=pkg_name,
            executable='mission_node',
            name='mission_node',
            output='screen'
        ),
        
        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        
        # Note: Teleop node is usually run in a separate terminal for input, 
        # but we could launch it here in a new xterm if needed. 
        # For now, we'll just launch the sim and rviz.
    ])
