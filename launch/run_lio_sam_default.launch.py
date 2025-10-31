import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'liloc'
    share_dir = get_package_share_directory(package_name)
    parameter_file = LaunchConfiguration('param_file')
    rviz_config_file = os.path.join(share_dir, 'launch', 'rviz', 'mapping.rviz')

    params_declare = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            share_dir, 'config', 'lio_sam_default.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    return LaunchDescription([
        params_declare,
        Node(
            package=package_name,
            executable=f'{package_name}_imuPreintegration',
            name=f'{package_name}_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package=package_name,
            executable=f'{package_name}_imageProjection',
            name=f'{package_name}_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package=package_name,
            executable=f'{package_name}_mapOptimization',
            name=f'{package_name}_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])