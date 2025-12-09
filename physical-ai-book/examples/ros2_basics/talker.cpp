// talker.cpp
// ROS 2 Publisher Example in C++
// Part of the Physical AI & Humanoid Robotics educational book

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node
{
public:
    TalkerNode()
    : Node("talker_node"), count_(0)
    {
        // Create a publisher for the 'chatter' topic with a queue size of 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // Create a timer to publish messages at regular intervals (500ms)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&TalkerNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Talker node started, publishing to /chatter topic");
    }

private:
    void timer_callback()
    {
        // Create a message
        auto message = std_msgs::msg::String();
        message.data = "Hello World: " + std::to_string(count_++);

        // Publish the message
        publisher_->publish(message);

        // Log the message being sent
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create and run the talker node
    rclcpp::spin(std::make_shared<TalkerNode>());

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}