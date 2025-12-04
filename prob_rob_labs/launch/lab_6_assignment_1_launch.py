import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'prob_rob_labs'
    try:
        pkg_share = get_package_share_directory(package_name)
        default_map_path = os.path.join(pkg_share, 'config', 'lab6_landmarks.yaml')
    except Exception as e:
        print(f"Error finding package share directory: {e}")
        default_map_path = ''

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map_path,
            description='Full path to the landmarks YAML file'),
        Node(
            package=package_name,
            executable='lab_6_assignment_1', 
            name='lab_6_assignment_1',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file': LaunchConfiguration('map_file')
            }]
        )
    ])