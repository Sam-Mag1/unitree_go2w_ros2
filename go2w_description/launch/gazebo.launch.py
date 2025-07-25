from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_go2w = FindPackageShare("go2w_description")
    xacro_file = PathJoinSubstitution([pkg_go2w, "urdf", "go2w_description.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_go2w, "config", "rviz_config.rviz"])
    controller_yaml = PathJoinSubstitution([pkg_go2w, "config", "joint_controller.yaml"])

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="default.world",
        description="World file name to load from the worlds directory"
    )
    
    gzserver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("gazebo_ros"), "/launch", "/gzserver.launch.py"
            ]),
            launch_arguments={
                "world": PathJoinSubstitution([
                    FindPackageShare("go2w_config"), "worlds", LaunchConfiguration("world")
                ]),
            }.items()
        )
    gzclient_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("gazebo_ros"), "/launch", "/gzclient.launch.py"
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
            "-z", "0.8",
            
        ],
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--param-file", controller_yaml],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_group_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_effort_controller', '--controller-manager', '/controller_manager', "--param-file", controller_yaml],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_velocity_controller', '--controller-manager', '/controller_manager', "--param-file", controller_yaml],
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

    return LaunchDescription([
        declare_world,
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        #ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_group_effort_controller_spawner,
        joint_group_velocity_controller_spawner,
        rviz_node
    ])
