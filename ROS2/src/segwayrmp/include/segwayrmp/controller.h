#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "segway_msgs/msg/chassis_mode_fb.hpp"
#include "segway_msgs/srv/ros_set_chassis_enable_cmd.hpp"

namespace segway {

class SegwayController : public rclcpp::Node {
public:
    explicit SegwayController(const rclcpp::NodeOptions & options);

private:
    void publish_cmd_vel();
    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void callback_chassis_mode(const segway_msgs::msg::ChassisModeFb::SharedPtr msg);
    bool initialize();
    bool enable();
    void disable();

    // Timestamp of the last command received
    std::chrono::time_point<std::chrono::steady_clock> last_command_update_;
    // Command velocity sent on timer
    geometry_msgs::msg::Twist cmd_vel_;
    // Current mode of chassis
    int chassis_mode_;
    // Publisher for command velocity after state handling
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    // Subscriber for command velocity input
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    // Subscriber for chassis mode
    rclcpp::Subscription<segway_msgs::msg::ChassisModeFb>::SharedPtr sub_chassis_mode_;
    // Segway client to enable/disable control
    rclcpp::Client<segway_msgs::srv::RosSetChassisEnableCmd>::SharedPtr enable_cli_;
    // 10 Hz timer that triggers publish command velocities
    rclcpp::TimerBase::SharedPtr timer_;
};

}
