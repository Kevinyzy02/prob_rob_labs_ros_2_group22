import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('forward_speed', default_value = '0.5'),
        Node(
            package='prob_rob_labs',
            executable='lab_2_question_4',
            name='lab_2_question_4',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'forward_speed': LaunchConfiguration('forward_speed')}]
        )
    ])
