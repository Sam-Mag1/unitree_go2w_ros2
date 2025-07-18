from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_go2w = FindPackageShare("go2w_description")
    xacro_file = PathJoinSubstitution([pkg_go2w, "urdf", "go2w_description.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_go2w, "config", "rviz_config.rviz"])
    controller_yaml = PathJoinSubstitution([pkg_go2w, "config", "ros2_control.yaml"])
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"
        ])
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file]),
                "use_sim_time": True
            }
        ]
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "go2w",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "1"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_group_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         controller_yaml,
    #         {"use_sim_time": True}
    #     ],
    #     output="screen"
    # )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_broadcaster_spawner,
        joint_group_effort_controller_spawner,
        joint_group_velocity_controller_spawner,
        rviz_node,
        #ros2_control_node,
    ])
