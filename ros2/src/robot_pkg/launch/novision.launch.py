import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the Parameter File
    # I'm doing the hard work of finding the path for you.
    pkg_share = get_package_share_directory('pkg')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Verify it actually exists (because I don't trust you)
    if not os.path.exists(config_file):
        print(f"⚠️ [WARNING] Could not find params.yaml at: {config_file}")
        print("I'm launching with default values, but don't come crying to me later.")

    return LaunchDescription([
        
        # --- 1. HARDWARE DRIVER ---
        # The 'driver' entry point from your setup.py
        Node(
            package='pkg',
            executable='driver',
            name='serial_driver',
            output='screen',
            emulate_tty=True,
            parameters=[config_file]  # Loads params.yaml
        ),


        # --- 3. PLANNER BRAIN ---
        # The 'planner' entry point
        Node(
            package='pkg',
            executable='planner',
            name='planner_node',
            output='screen',
            parameters=[config_file]
        ),
    ])