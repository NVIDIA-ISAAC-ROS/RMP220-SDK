#include "rclcpp/rclcpp.hpp"
#include "segwayrmp/robot.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    robot::Chassis sbv(options);
    rclcpp::spin(sbv.node);
    rclcpp::shutdown();
    return 0;
}