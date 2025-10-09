import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('frame_id', default_value='map',
                              description='Frame of reference for ground truth messages'),
        Node(
            package='prob_rob_labs',
            executable='lab_4_assignment_1',
            name='lab_4_assignment_1',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'frame_id': LaunchConfiguration('frame_id')}]
        )
    ])
