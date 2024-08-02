#include "segwayrmp/controller.h"
#include "comm_ctrl_navigation.h"


namespace segway {

SegwayController::SegwayController(const rclcpp::NodeOptions & options)
: Node("segway_controller", options) {
    cmd_vel_ = geometry_msgs::msg::Twist();
    last_command_update_ = std::chrono::steady_clock::now();
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("out_cmd_vel", 10);
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "in_cmd_vel", 10, 
        std::bind(&SegwayController::callback_cmd_vel, this, std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&SegwayController::publish_cmd_vel, this)
    );
    using rclcpp::contexts::get_global_default_context;
    get_global_default_context()->add_pre_shutdown_callback(
        [this]() {
        auto message = geometry_msgs::msg::Twist();
        pub_cmd_vel_->publish(message);
        exit();
    });

    // Initialize segway
    set_comu_interface(comu_can);
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize host control, check if chassis is connected");
    }
    if (!enable()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enable host control at startup");
    }
}

void SegwayController::publish_cmd_vel() {
    // Re-enable when chassis enters a locked state
    if (get_chassis_mode() == 0) {
        // exit();
        // initialize();
        if (!enable()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable host control at runtime.");
            return;
        }
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

bool SegwayController::initialize() {
    RCLCPP_INFO(this->get_logger(), "Initializing segway");
    if (init_control_ctrl() == -1) {
        return false;
    }
    return true;
}

bool SegwayController::enable() {
    RCLCPP_INFO(this->get_logger(), "Enabling segway");
    if (set_enable_ctrl(1) != 0) {
        return false;
    }
    return true;
}

void SegwayController::exit() {
    RCLCPP_INFO(this->get_logger(), "Exiting segway");
    set_enable_ctrl(0);
    exit_control_ctrl();
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(segway::SegwayController)
