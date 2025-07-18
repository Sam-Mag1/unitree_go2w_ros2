from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    simulation_launcher = os.path.join(get_package_share_directory('go2w_description'), 'launch', 'gazebo.launch.py')

    joint_publisher_launcher = os.path.join(get_package_share_directory('go2w_control'), 'launch', 'joint_publisher.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launcher)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joint_publisher_launcher)
        ),
    ])