import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    laser_z = LaunchConfiguration('laser_z')

    pkg_share = get_package_share_directory('slam_bringup')
    slam_config = os.path.join(pkg_share, 'config',
                               'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Start RViz'),
        DeclareLaunchArgument(
            'laser_z', default_value='0.15',
            description='LiDAR height above base_link (m)'),

        # ---- odometry + scan TCP bridge (ESP32) ----
        Node(
            package='slam_bringup', executable='translater_node',
            name='translater_node', output='screen',
        ),

        # ---- static TF: base_link -> laser_link ----
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_laser_link',
            arguments=['0', '0', laser_z, '0', '0', '0',
                       'base_link', 'laser_link'],
        ),

        # ---- SLAM Toolbox (map -> odom) ----
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[slam_config],
        ),

        # ---- RViz (optional) ----
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen', condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
        ),
    ])
