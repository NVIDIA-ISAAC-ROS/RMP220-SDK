#include "segwayrmp/controller.h"
#include "comm_ctrl_navigation.h"


namespace segway {

SegwayController::SegwayController(const rclcpp::NodeOptions & options)
: Node("segway_controller", options) {
    cmd_vel_ = geometry_msgs::msg::Twist();
    last_command_update_ = std::chrono::steady_clock::now();
    chassis_mode_ = -1;
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("out_cmd_vel", 10);
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "in_cmd_vel", 10, 
        std::bind(&SegwayController::callback_cmd_vel, this, std::placeholders::_1)
    );
    sub_chassis_mode_ = this->create_subscription<segway_msgs::msg::ChassisModeFb>(
        "chassis_mode_fb", 10, std::bind(&SegwayController::callback_chassis_mode,
        this, std::placeholders::_1));
    enable_cli_ = this->create_client<segway_msgs::srv::RosSetChassisEnableCmd>("set_chassis_enable");
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&SegwayController::publish_cmd_vel, this)
    );
    // Shutdown segway by publishing a 0 message and then disabling control
    using rclcpp::contexts::get_global_default_context;
    get_global_default_context()->add_pre_shutdown_callback(
        [this]() {
        auto message = geometry_msgs::msg::Twist();
        pub_cmd_vel_->publish(message);
        disable();
    });
}

void SegwayController::publish_cmd_vel() {
    // Re-enable when chassis enters a locked state
    if (chassis_mode_ < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Segway chassis mode not received yet. Waiting.");
        return;
    } else if (!enable_cli_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Segway enable service is not ready yet. Waiting.");
        return;
    } else if (chassis_mode_ == 0) {
        auto enable_request = std::make_shared<segway_msgs::srv::RosSetChassisEnableCmd::Request>();
        enable_request->ros_set_chassis_enable_cmd = 1;
        // TODO: should we wait for a future result and check that it was a success?
        auto result = this->enable_cli_->async_send_request(enable_request);
    }
    // Reset command velocity to 0 if it hasn't been updated recently
    auto time_since_last_command =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_command_update_);
    if (time_since_last_command.count() > 500 && cmd_vel_.linear.x != 0 && cmd_vel_.angular.z != 0) {
        RCLCPP_WARN(this->get_logger(),
            "No command received for %ldms, stopping Segway", time_since_last_command.count());
        cmd_vel_ = geometry_msgs::msg::Twist();
    }
    auto message = geometry_msgs::msg::Twist();
    message = cmd_vel_;
    pub_cmd_vel_->publish(message);
}

void SegwayController::callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_command_update_ = std::chrono::steady_clock::now();
    cmd_vel_ = *msg;
}

void SegwayController::callback_chassis_mode(const segway_msgs::msg::ChassisModeFb::SharedPtr msg) {
    chassis_mode_ = msg->chassis_mode;
}

void SegwayController::disable() {
    RCLCPP_INFO(this->get_logger(), "Disabling segway control");
    auto enable_request = std::make_shared<segway_msgs::srv::RosSetChassisEnableCmd::Request>();
    enable_request->ros_set_chassis_enable_cmd = 0;
    auto result = this->enable_cli_->async_send_request(enable_request);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(segway::SegwayController)
