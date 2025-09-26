from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch argument for joystick device
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    # Resolve path to teleop config file
    config_file = os.path.join(
        get_package_share_directory('prob_rob_labs'),
        'config',
        'joystick.yaml'
    )

    # joy_node with parameters
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device': LaunchConfiguration('dev'),
            'deadzone': 0.3,
            'autorepeat_rate': 20.0
        }],
        output='screen'
    )

    # teleop_twist_joy node with config file
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        dev_arg,
        joy_node,
        teleop_node
    ])
