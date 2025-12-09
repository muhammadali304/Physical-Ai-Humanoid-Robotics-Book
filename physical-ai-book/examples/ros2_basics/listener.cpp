// listener.cpp
// ROS 2 Subscriber Example in C++
// Part of the Physical AI & Humanoid Robotics educational book

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode()
    : Node("listener_node")
    {
        // Create a subscription to the 'chatter' topic with a queue size of 10
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                // Log the received message
                RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "Listener node started, subscribing to /chatter topic");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create and run the listener node
    rclcpp::spin(std::make_shared<ListenerNode>());

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();

    return 0;
}