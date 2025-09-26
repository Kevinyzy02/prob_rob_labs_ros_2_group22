import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_dic = {}
    return LaunchDescription([
        Node(
            package='prob_rob_labs',
            executable='image_mean_feature_x',
            name='image_mean_feature_x'

        )
    ])
