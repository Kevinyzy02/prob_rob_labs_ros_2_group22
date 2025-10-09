import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('frame_id', default_value='odom'),
        DeclareLaunchArgument('linear_x', default_value='0.2'),
        DeclareLaunchArgument('angular_z', default_value='0.0'),

        Node(
            package='prob_rob_labs',
            executable='lab_4_assignment_2',
            name='lab_4_assignment_2',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'frame_id': LaunchConfiguration('frame_id')}, {'linear_x': LaunchConfiguration('linear_x')}, {'angular_z': LaunchConfiguration('angular_z')}]
        )
    ])
