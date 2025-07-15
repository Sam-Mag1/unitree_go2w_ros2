from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('go2w_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'go2w_description.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'user_debug',
            default_value='false',
            description='Enable debug mode'
        ),

        # robot_description param loaded from URDF file content
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()},
                        {'publish_frequency': 1000.0}]
        ),

        # joint_state_publisher_gui node with use_gui = True
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_gui': True}]
        ),

        # RViz2 with config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])