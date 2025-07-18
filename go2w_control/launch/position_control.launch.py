import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    simulation_launcher = os.path.join(get_package_share_directory('go2w_description'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([simulation_launcher])
        ),

        Node(
            package='go2w_control',
            executable='position_control',
            name='position_control_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])