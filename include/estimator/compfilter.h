#ifndef COMPFILTER_H
#define COMPFILTER_H

#include <Eigen/Core> 
#include <Eigen/Geometry>
#include <vector>

namespace cf {

struct CompFilterParams {
    Eigen::Matrix<double, 3, 1> gyroBias0;
    double gyroGain;
    double accelGain;
    double biasGain;
    
};

class CompFilter
{
    public:
        CompFilter(CompFilterParams cfParams);
        ~CompFilter();

        bool imu_update(double dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel);

        Eigen::Quaterniond get_quat() const;
        Eigen::Vector3d get_bias() const;
    
    protected:
        CompFilterParams cfParams;
        Eigen::Quaterniond quat;
        Eigen::Vector3d gyroBias;
        Eigen::Vector3d gyroAvg;
        double gravity;
        int gyroAvgCount{0};
        bool getInitBias{true};

        void propogate_quaternion(double dt, const Eigen::Quaterniond& quatDot);
        void average_stationary_bias(const Eigen::Vector3d& gyro);
        Eigen::Vector3d vee(const Eigen::Matrix3d& m);
        void set_initial_state();
};

}

#endif