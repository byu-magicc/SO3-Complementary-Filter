#ifndef CF_ROS_H
#define CF_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "compfilter.h"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "ublox_read_2/msg/rel_pos.hpp"


namespace cf {

class CF_ROS : public rclcpp::Node
{
public: 
    CF_ROS();

private:

    std::unique_ptr<CompFilter> compFilter_;

    bool firstImu{true};
    double imuTimePrev{0};

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<ublox_read_2::msg::RelPos>::SharedPtr compassSub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitudePub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gyroBiasPub_;

    // cf_ros_init
    void init_subs_and_pubs();
    void init_params();
    void instantiate_estimator();
    void init_transforms();

    // publish
    void publish_estimate() const;

    // sensor callback
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void compass_callback(const ublox_read_2::msg::RelPos::SharedPtr msg);

    // utils
    // Eigen::Matrix4d get_transform_from_odom_msg(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    // Eigen::Vector3d get_position_from_odom_msg(const geometry_msgs::msg::Point pROS) const;
    // Eigen::Matrix3d get_rotation_matrix_from_quaternion(const geometry_msgs::msg::Quaternion qROS);
    // Eigen::Matrix4d get_transform(const Eigen::Matrix3d R, Eigen::Vector3d p) const;
    // Eigen::Vector3d copy_vector3_data(const std::array<double, 3> data);
    Eigen::Vector3d get_gyro_from_imu(const sensor_msgs::msg::Imu::SharedPtr msg) const;
    Eigen::Vector3d get_accel_from_imu(const sensor_msgs::msg::Imu::SharedPtr msg) const;
    // void set_ref_lla(const ublox_read_2::msg::PosVelEcef::SharedPtr msg);
    // Eigen::Vector3d ecef2ned_vel(const ublox_read_2::msg::PosVelEcef::SharedPtr msg);
    // geometry_msgs::msg::Point get_estimator_position() const;
    geometry_msgs::msg::Quaternion get_estimator_quaternion() const;
    geometry_msgs::msg::Vector3 get_estimator_bias() const;
    // geometry_msgs::msg::Vector3 get_estimator_velocity() const;
};

}

#endif