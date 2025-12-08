import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='prob_rob_labs',
            executable='final_project',
            name='odometry_ekf',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'filter_type': 'ekf'}
            ],
            remappings=[
                ('/ekf_odom', '/odom_ekf') 
            ]
        ),

        Node(
            package='prob_rob_labs',
            executable='final_project',
            name='odometry_ukf',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'filter_type': 'ukf'}
            ],
            remappings=[
                ('/ekf_odom', '/odom_ukf')
            ]
        ),

        Node(
            package='prob_rob_labs',
            executable='lab_4_assignment_1',
            name='ground_truth',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['odom'] 
        )
    ])