from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument



def generate_launch_description():

    rviz_config_path = os.path.join(
    get_package_share_directory('odometer'),
    'config',
    'odometry_view.rviz'
)
    return LaunchDescription([

        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path
        ),
        Node(
            package='odometer',
            namespace='taso',
            executable='odometer',
            name='OdometerNode'
        ),
        Node(
            package='odometer',
            namespace='taso',
            executable='square',
            name='MotionPlanner',
            parameters=[{'mode': 'square'}]
            )
        ,
        Node(
            package='rviz2',
            namespace='taso',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')]   
        )
    ])