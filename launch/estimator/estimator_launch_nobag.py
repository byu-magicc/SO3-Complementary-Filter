import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from datetime import datetime
from pathlib import Path


def generate_launch_description():

    now = datetime.now()

    config_file = LaunchConfiguration('config_file')
    config_path = LaunchConfiguration('config_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='config_file',
            default_value='params.yaml'
        ),
        DeclareLaunchArgument(
            name='config_path',
            default_value=[get_package_share_directory('so3_cf'), '/params/', config_file]
        ),
        DeclareLaunchArgument(
            name='record_bag',
            default_value='True'
        ),

        Node(
            package = 'so3_cf',
            executable = 'cf_ros',
            name = 'cf_ros',
            output = 'screen',
            parameters = [config_path]
        )
    ])
 