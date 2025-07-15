// #include <rclcpp/rclcpp.hpp>
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
// #include "trajectory_msgs/msg/joint_trajectory_point.hpp"
// #include <cmath>
// #include <iostream>
// #include <chrono>
// #include <vector>

// class JointPublisher : public rclcpp::Node {
// public:
//     JointPublisher() : Node("joint_publisher") {
//         publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),
//             std::bind(&JointPublisher::publish_joint_trajectory, this));
//     }

// private:
//     void publish_joint_trajectory() {
//         auto message = trajectory_msgs::msg::JointTrajectory();
//         message.joint_names = joint_list;

//         auto point = trajectory_msgs::msg::JointTrajectoryPoint();

//         for (size_t i = 0; i < joint_list.size(); ++i) {
//             if (i % 4 == 1) { // For thigh joints
//                 positions[i] = 1.0143535137176514 + std::sin(count_ * 0.1);
//             } else if (i % 4 == 2) { // For calf joints
//                 positions[i] = -2.0287070274353027 - std::sin(count_ * 0.1);
//             }
//         }
//         point.positions = positions; // Use the modified positions vector

//         point.time_from_start = rclcpp::Duration::from_seconds(0.5);
//         message.points.push_back(point);
//         publisher_->publish(message);
//         RCLCPP_INFO(this->get_logger(), "Published joint trajectory: hip = %.2f, thight = %.2f, calf = %.2f, foot = %.2f", 
//                     point.positions[0], point.positions[1], point.positions[2], point.positions[3]);
//         count_ +=1;
//     }

//     std::vector<std::string> joint_list = {
//         "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FL_foot_joint",
//         "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FR_foot_joint",
//         "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RL_foot_joint",
//         "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RR_foot_joint"
//     };
//     std::vector<double> positions = {
//         0.0, 1.0143535137176514, -2.0287070274353027, 0.0,
//         0.0, 1.0143535137176514, -2.0287070274353027, 0.0,
//         0.0, 1.0143535137176514, -2.0287070274353027, 0.0,
//         0.0, 1.0143535137176514, -2.0287070274353027, 0.0
//     };


//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
//     int count_ = 0;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<JointPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }



#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <chrono>

class EffortPublisher : public rclcpp::Node {
public:
    EffortPublisher() : Node("effort_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_trajectory_controller/commands", 10);
    }

    void publish_efforts(const std::vector<double>& efforts) {
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = efforts;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published effort command from user input");
    }

private:
    const size_t num_joints = 16;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EffortPublisher>();

    std::vector<double> efforts(16, 0.0);

    while (rclcpp::ok()) {
        size_t index;
        double value;
        std::cout << "Enter joint index (0-15) and value (or -1 to quit): ";
        std::cin >> index;
        if (!std::cin || index == static_cast<size_t>(-1)) break;
        if (index >= efforts.size()) {
            std::cout << "Invalid index. Try again.\n";
            continue;
        }
        std::cin >> value;
        if (!std::cin) break;
        efforts[index] = value;
        node->publish_efforts(efforts);
    }

    rclcpp::shutdown();
    return 0;
}
