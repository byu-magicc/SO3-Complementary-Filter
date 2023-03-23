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
    double compassGain;
    
};

class CompFilter
{
    public:
        CompFilter(CompFilterParams cfParams);
        ~CompFilter();

        bool imu_update(double dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel);
        void compass_update(double zCompass);
        Eigen::Quaterniond get_quat() const;
        Eigen::Vector3d get_bias() const;
    
    protected:
        CompFilterParams cfParams;
        Eigen::Quaterniond quat;
        Eigen::Vector3d gyroBias;
        Eigen::Vector3d gyroAvg;
        double gravity;
        int gyroAvgCount;
        bool getInitBias;

        bool compassReceived;
        double compass;

        void propogate_quaternion(double dt, const Eigen::Quaterniond& quatDot);
        void average_stationary_bias(const Eigen::Vector3d& gyro);
        Eigen::Vector3d vee(const Eigen::Matrix3d& m);
        void set_initial_state();
        Eigen::Vector3d quat_to_euler();
        Eigen::Matrix3d euler_to_rotation(double phi, double theta, double psi);

};

}

#endif