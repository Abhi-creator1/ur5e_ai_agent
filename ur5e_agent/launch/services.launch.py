"""
Launch file for UR5e AI agent services
Starts the cartesian motion server and controller switcher
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for UR5e AI agent services"""
    
    # Cartesian motion control service
    cartesian_motion_server = Node(
        package='ur5e_agent',
        executable='cartesian_motion_server.py',
        name='cartesian_motion_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    
    # Controller switching service  
    controller_switcher = Node(
        package='ur5e_agent',
        executable='controller_switcher.py',
        name='controller_switcher', 
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        cartesian_motion_server,
        controller_switcher,
        
    ])
