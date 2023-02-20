#include "cf_ros.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cf::CF_ROS>());
    rclcpp::shutdown();
    return 0;

}
