#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    prob_rob_vision_dir = get_package_share_directory('prob_rob_vision')
    vision_green_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_vision_dir, 'launch', 'video_processor_launch.py')
        ),
        launch_arguments={
            'min_h': '118',
            'max_h': '122',
            'run_color_filter': 'true',
            'max_features' : '16',
            'goodfeature_corners_topic': '/vision_green/corners',
            'goodfeature_image_topic': '/vision_green/image_raw',
            'gray_image_topic': '/vision/green/gray_image_raw',
            'node_name': 'green_video_processor'
        }.items()
    )
    vision_red_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_vision_dir, 'launch', 'video_processor_launch.py')
        ),
        launch_arguments={
            'min_h': '3',
            'max_h': '7',
            'run_color_filter': 'true',
            'max_features' : '16',
            'goodfeature_corners_topic': '/vision_red/corners',
            'goodfeature_image_topic': '/vision_red/image_raw',
            'gray_image_topic': '/vision/red/gray_image_raw',
            'node_name': 'red_video_processor'
        }.items()
    )
    vision_yellow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_vision_dir, 'launch', 'video_processor_launch.py')
        ),
        launch_arguments={
            'min_h': '58',
            'max_h': '62',
            'run_color_filter': 'true',
            'max_features' : '16',
            'goodfeature_corners_topic': '/vision_yellow/corners',
            'goodfeature_image_topic': '/vision_yellow/image_raw',
            'gray_image_topic': '/vision/yellow/gray_image_raw',
            'node_name': 'yellow_video_processor'
        }.items()
    )
    vision_magenta_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_vision_dir, 'launch', 'video_processor_launch.py')
        ),
        launch_arguments={
            'min_h': '298',
            'max_h': '302',
            'run_color_filter': 'true',
            'max_features' : '16',
            'goodfeature_corners_topic': '/vision_magenta/corners',
            'goodfeature_image_topic': '/vision_magenta/image_raw',
            'gray_image_topic': '/vision/magenta/gray_image_raw',
            'node_name': 'magenta_video_processor'
        }.items()
    )
    vision_cyan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_vision_dir, 'launch', 'video_processor_launch.py')
        ),
        launch_arguments={
            'min_h': '178',
            'max_h': '182',
            'run_color_filter': 'true',
            'max_features' : '16',
            'goodfeature_corners_topic': '/vision_cyan/corners',
            'goodfeature_image_topic': '/vision_cyan/image_raw',
            'gray_image_topic': '/vision/cyan/gray_image_raw',
            'node_name': 'cyan_video_processor'
        }.items()
    )

    return LaunchDescription([
        vision_green_launch,
        vision_red_launch,
        vision_yellow_launch,
        vision_magenta_launch,
        vision_cyan_launch
    ])
