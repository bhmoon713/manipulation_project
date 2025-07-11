from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('object_detection'),
        'rviz',
        'sensor_data_real.rviz'
    )

    return LaunchDescription([
        Node(
            package='object_detection',
            executable='object_detection_real',
            name='object_detection_real',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        # Node(
        #     package='object_detection',
        #     executable='static_transform_publisher_real',
        #     name='static_transform_publisher_real',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}]
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])