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

    params_file = LaunchConfiguration('params_file')
    params_path = LaunchConfiguration('params_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value='params.yaml'
        ),
        DeclareLaunchArgument(
            name='params_path',
            default_value=[get_package_share_directory('so3_cf'), '/params/', params_file]
        ),
        DeclareLaunchArgument(
            name='record_bag',
            default_value='True'
        ),
        DeclareLaunchArgument(
            name='bag_filename',
            default_value= str(Path.home()) + '/data/cf/cf_' + now.strftime("%m-%d-%Y_%H:%M:%S")
        ),

        Node(
            package = 'so3_cf',
            executable = 'cf_ros',
            name = 'cf_ros',
            output = 'screen',
            parameters = [params_path]
        ),

        ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('record_bag')
                ])
            ),
            cmd=[
                'ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_filename'),
                '/cf/bias',
                '/cf/attitude',
                '/rhodey/pose_ned',
                '/camera/imu'
                ],
        )
    ])
 