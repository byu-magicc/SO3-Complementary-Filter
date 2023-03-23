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
    imu_topic = LaunchConfiguration('imu_topic')

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
        DeclareLaunchArgument(
            name='bag_name',
            default_value= str(Path.home()) + '/data/cf/cf_' + now.strftime("%m-%d-%Y_%H:%M:%S")
        ),
        DeclareLaunchArgument(
            name='imu_topic',
            default_value= '/vectornav/imu_uncompensated'
            # default_value= '/camera/imu'
        ),

        Node(
            package = 'so3_cf',
            executable = 'cf_ros',
            name = 'cf_ros',
            output = 'screen',
            parameters = [config_path],
            remappings = [('/imu', imu_topic)]
        ),

        ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('record_bag')
                ])
            ),
            cmd=[
                'ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_name'),
                '/cf/bias',
                '/cf/attitude',
                '/rhodey/pose_ned',
                '/boat_landing_platform/pose_ned',
                '/camera/imu',
                '/vectornav/imu_uncompensated'
                ],
        )
    ])
 