#include "cf_ros.h"
#include <iostream>

using std::placeholders::_1;

namespace cf
{
    void CF_ROS::init_subs_and_pubs()
    {

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("/camera/imu", 10, std::bind(&CF_ROS::imu_callback, this, _1));
  
        // Publishers
        attitudePub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("/cf/attitude", 1);
        gyroBiasPub_ = this->create_publisher<sensor_msgs::msg::Imu>("/cf/bias", 1);
    }

    void CF_ROS::init_params()
    {
        std::map<std::string, std::vector<double>> vector_param;
        vector_param["gyroBias0"] = {0.1, 0.1, 0.1};

        this->declare_parameters("", vector_param);
        this->declare_parameter("gyroGain", 0.1);
        this->declare_parameter("accelGain", 0.1);
        this->declare_parameter("biasGain", 0.1);

    }

    void CF_ROS::instantiate_estimator()
    {
        CompFilterParams cfParams;

        cfParams.gyroGain = this->get_parameter("gyroGain").as_double();
        cfParams.accelGain = this->get_parameter("accelGain").as_double();
        cfParams.biasGain = this->get_parameter("biasGain").as_double();
        std::cerr << "GAINS\n" << "Gyro: " << cfParams.gyroGain<< " Accel: " << cfParams.accelGain<< " Bias: " << cfParams.biasGain << "\n\n";

        std::vector<double> paramPlaceholder;

        paramPlaceholder = this->get_parameter("gyroBias0").as_double_array();
        cfParams.gyroBias0 = Eigen::Map<Eigen::Vector3d>(paramPlaceholder.data(), paramPlaceholder.size());
        std::cerr << "BIAS0 \n" << cfParams.gyroBias0 << "\n"; 

        compFilter_ = std::make_unique<CompFilter>(cfParams);

    }

}