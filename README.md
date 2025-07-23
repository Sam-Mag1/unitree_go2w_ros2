# unitree_go2w_ros2

This repository allows the user to use ROS2 to create a digital twin of the Unitree Go2W in Gazebo.

## Quick Start

The project was tested with ROS2 Humble and Ubuntu 22.04. 

1. **Clone the repository**:
    ```bash
        git clone https://github.com/Sam-Mag1/unitree_go2w_ros2.git
        cd unitree_go2w_ros2
    ```
2. **Install dependencies**:
    ```bash
        sudo apt-get install python3-rosdep
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
    ```
3. **Build and source the workspace**:
    ```bash
        colcon build --symlink-install
        source install/setup.bash
    ```
4. **Launch the simulation with Controls**:
    ```bash
        ros2 launch go2w_control gazebo_with_control.launch.py
    ```

From this point, you have the Rviz2 interface open, which shows the robot model and the joint states, and the Gazebo simulation running with the Unitree Go2W model.

You can control the robot using an IHM with sliders. The robot is controlled in position for the legs, and in velocity for the wheels.

## __/!\\ Important Note /!\\__
There is no Hihg-Level control implemented yet. The robot can move each joint independently, but it does not have any high-level commands like "walk", "stand up", etc. 

At the moment, I am not sure that the torque have a "limit". The command is sent in effort to the robot to reach the desired position sent by the user, but the torque may be really high, which is not realistic. I will check how to limit the torque as soon as possible.

## To-Do List

- [x] Spawn the robot in Gazebo and link Rviz2 to the simulation
- [x] Be able to control the robot in effort mode
- [x] Add a very basic GUI to test the robot control
- [x] Be able to control the robot in position mode (without effort limitation)
- [x] Be able to control the robot in position mode (with an effort limitation)
- [ ] Be able to control the robot with high-level commands (e.g. walk, stand up, etc.)

*This To-Do list is not exhaustive and will be updated as the project progresses.*