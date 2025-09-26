#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    prob_rob_labs_dir = get_package_share_directory('prob_rob_labs')

    run_vision_processor = LaunchConfiguration('run_vision_processor', default='false')
    run_door_opener = LaunchConfiguration('run_door_opener', default='false')

    declare_run_vision_processor_arg = DeclareLaunchArgument(
        'run_vision_processor',
        default_value='false',
        description='Whether to run the vision processing nodes'
    )

    declare_run_door_opener_arg = DeclareLaunchArgument(
        'run_door_opener',
        default_value='false',
        description='Whether to run the flaky door opener node'
    )

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_labs_dir, 'launch', 'turtlebot3_and_door_launch.py')
        ),
        launch_arguments={
            'world': 'door_and_firetruck.world',
            'run_vision_processor': run_vision_processor,
            'run_door_opener': run_door_opener
        }.items()
    )

    return LaunchDescription([
        declare_run_vision_processor_arg,
        declare_run_door_opener_arg,
        main_launch
    ])
