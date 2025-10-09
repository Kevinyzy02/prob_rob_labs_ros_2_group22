import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('test_axis', default_value='angular'),
        DeclareLaunchArgument('step_value', default_value='0.3'),
        DeclareLaunchArgument('zero_time', default_value='2.0'),
        DeclareLaunchArgument('hold_time', default_value='6.0'),
        DeclareLaunchArgument('rate_hz', default_value='20.0'),

        Node(
            package='prob_rob_labs',
            executable='lab_4_assignment_3',
            name='lab_4_assignment_3',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {
                'test_axis': LaunchConfiguration('test_axis'),
                'step_value': LaunchConfiguration('step_value'),
                'zero_time': LaunchConfiguration('zero_time'),
                'hold_time': LaunchConfiguration('hold_time'),
                'rate_hz': LaunchConfiguration('rate_hz'),
            }]
        )
    ])
