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
    except Exception:
        default_map_path = ''

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_file', default_value=default_map_path),

        Node(
            package=package_name,
            executable='lab_5_assignment_2', 
            name='vision_processing_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(
            package=package_name,
            executable='lab_6_assignment_2',
            name='ekf_localization_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file': LaunchConfiguration('map_file'),
            }]
        ),

        Node(
            package=package_name,
            executable='lab_6_assignment_5',
            name='map_odom_broadcaster',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'ekf.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])