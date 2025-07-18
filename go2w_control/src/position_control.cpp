#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <chrono>

class EffortPublisher : public rclcpp::Node {
    public:
        EffortPublisher() : Node("effort_publisher") {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/joint_group_effort_controller/commands", 10);
            subscription_trajectory_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "/joint_trajectory_controller/joint_trajectory", 10,
                std::bind(&EffortPublisher::trajectory_callback, this, std::placeholders::_1));
            subscription_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&EffortPublisher::state_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&EffortPublisher::timer_callback, this));
        }

        void publish_efforts(const std::vector<double>& efforts) {
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = efforts;
            publisher_->publish(message);
            // RCLCPP_INFO(this->get_logger(), "Published effort command from position input");
        }

        void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
            this->desired_position = msg->points[0].positions;
            // RCLCPP_INFO(this->get_logger(), "Received new trajectory command");
        }

        void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->current_position = msg->position;
            // RCLCPP_INFO(this->get_logger(), "Received current joint states");
        }

        void timer_callback() {
            std::vector<double> efforts(num_joints, 0.0);
            for (size_t i = 0; i < num_joints; ++i) {
                double error = fmod(desired_position[i] - current_position[i] + M_PI, 2 * M_PI) - M_PI;
                efforts[i] = std::clamp(error * kp[i], -effort_limit[i], effort_limit[i]);
                 if (i==1) RCLCPP_INFO(this->get_logger(), "Joint %zu: Desired Position = %f, Current Position = %f, Error = %f, Effort = %f",
                            i, desired_position[i], current_position[i], error, efforts[i]);
            }
            publish_efforts(efforts);
        }

    private:
        const size_t num_joints = 16;
        std::vector<double> desired_position = {
            0.0, 1.0143535137176514, -2.0287070274353027, 0.0,
            0.0, 1.0143535137176514, -2.0287070274353027, 0.0,
            0.0, 1.0143535137176514, -2.0287070274353027, 0.0,
            0.0, 1.0143535137176514, -2.0287070274353027, 0.0
        };
        std::vector<double> current_position = std::vector<double>(num_joints, 0.0);
        std::vector<double> effort_limit = std::vector<double>(num_joints, 45.0);
        std::vector<double> kp = std::vector<double>(num_joints, 10);
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_trajectory_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EffortPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
