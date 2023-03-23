#include <cf_ros.h>
#include <string.h>
#include <iostream>

namespace cf
{

    void CF_ROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        // RCLCPP_INFO(this->get_logger(), "In rover imu");
        double imuTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double dt = imuTime - imuTimePrev; 
        imuTimePrev = imuTime;
        
        if (firstImu)
        {
            firstImu = false;
            return;
        }
        
        bool success = compFilter_->imu_update(dt, get_gyro_from_imu(msg), get_accel_from_imu(msg));

        if (success) publish_estimate();
    }
    
    void CF_ROS::compass_callback(const ublox_read_2::msg::RelPos::SharedPtr msg)
    {
        compFilter_->compass_update(msg->rel_pos_heading);
    }
}