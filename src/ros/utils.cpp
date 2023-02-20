#include "cf_ros.h"

namespace cf {


geometry_msgs::msg::Quaternion CF_ROS::get_estimator_quaternion() const {

    Eigen::Quaterniond estimatorQuat = compFilter_->get_quat();

    geometry_msgs::msg::Quaternion quaternion;

    quaternion.x = estimatorQuat.x();
    quaternion.y = estimatorQuat.y();
    quaternion.z = estimatorQuat.z();
    quaternion.w = estimatorQuat.w();

    return quaternion;
}

geometry_msgs::msg::Vector3 CF_ROS::get_estimator_bias() const {

    Eigen::Vector3d estimatorBias = compFilter_->get_bias();

    geometry_msgs::msg::Vector3 bias;

    bias.x = estimatorBias(0);
    bias.y = estimatorBias(1);
    bias.z = estimatorBias(2);

    return bias;
}
Eigen::Vector3d CF_ROS::get_gyro_from_imu(const sensor_msgs::msg::Imu::SharedPtr msg) const {
    Eigen::Vector3d gyro;

    gyro.x() = msg->angular_velocity.x;
    gyro.y() = msg->angular_velocity.y;
    gyro.z() = msg->angular_velocity.z;

    return gyro;
}

Eigen::Vector3d CF_ROS::get_accel_from_imu(const sensor_msgs::msg::Imu::SharedPtr msg) const {
    Eigen::Vector3d accel;
    
    accel(0) = msg->linear_acceleration.x;
    accel(1) = msg->linear_acceleration.y;
    accel(2) = msg->linear_acceleration.z;

    return accel;
}

}  // namespace cf
