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
4. **Launch the simulation**:
    ```bash
        ros2 launch go2w_description go2w_gazebo.launch.py
    ```
5. **Test the controls**:
    In a new terminal:
    ```bash*
        source install/setup.bash
        ros2 run go2w_description joint_publisher
    ```

From this point, you have the Rviz2 interface open, which shows the robot model and the joint states, and the Gazebo simulation running with the Unitree Go2W model.

You can control the robot using a command line interface in the terminal where you launched the `joint_publisher`.

## __/!\\ Important Note /!\\__
The current implementation is not fully functional. The robot is controlled in effort mode, but he can't be controlled in position mode yet (so it can't stand up). 

If you want to test another command protocol, you can change the C++ code `unitree_go2w_ros2/go2w_description/src/joint_publisher.cpp` or add a new executable.

## To-Do List

- [x] Spawn the robot in Gazebo and link Rviz2 to the simulation
- [x] Be able to control the robot in effort mode
- [x] Add a very basic GUI to test the robot control
- [ ] Be able to control the robot in position mode (with an effort limitation)
- [ ] Be able to control the robot with high-level commands (e.g. walk, stand up, etc.)

*This To-Do list is not exhaustive and will be updated as the project progresses.*