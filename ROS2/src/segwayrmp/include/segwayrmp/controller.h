#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace segway {

class SegwayController : public rclcpp::Node {
public:
    explicit SegwayController(const rclcpp::NodeOptions & options);

private:
    void publish_cmd_vel();
    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool initialize();
    bool enable();
    void exit();

    // Timestamp of the last command received
    std::chrono::time_point<std::chrono::steady_clock> last_command_update_;
    // Command velocity sent on timer
    geometry_msgs::msg::Twist cmd_vel_;
    // Publisher for command velocity after state handling
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    // Subscriber for command velocity input
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    // 10 Hz timer that triggers publish command velocities
    rclcpp::TimerBase::SharedPtr timer_;
};

}
