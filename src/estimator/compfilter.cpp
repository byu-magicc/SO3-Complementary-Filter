#include "compfilter.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

namespace cf {

CompFilter::CompFilter(CompFilterParams params) 
    : cfParams{params}, gravity{9.81}
{
    set_initial_state();
}

CompFilter::~CompFilter() {}

void CompFilter::set_initial_state()
{
    quat = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    gyroBias = cfParams.gyroBias0;
    gyroAvg = Eigen::Vector3d::Zero();
}


void CompFilter::average_stationary_bias(const Eigen::Vector3d& gyro)
{
    int iters = 1000;
    if (gyroAvgCount < iters)
    {
        gyroAvg = gyroAvg+gyro;
        gyroAvgCount++;
    }
    else
    {
        getInitBias = false;
        gyroBias = gyroAvg/iters;
        std::cout << gyroBias;
    }
}

bool CompFilter::imu_update(double dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel)
{
    if(getInitBias)
    {
        average_stationary_bias(gyro);
        return false;
    }
    
    Eigen::Matrix3d R = quat.toRotationMatrix();
    Eigen::Vector3d Rz = R.transpose().block<3,1>(0,2);
    Eigen::Matrix3d alphaM = cfParams.accelGain/(gravity*gravity) * (accel * Rz.transpose() - Rz * accel.transpose()); //accel gain
    Eigen::Vector3d alpha = - vee(alphaM);
    
    Eigen::Vector3d omegaBar = gyro - gyroBias + cfParams.gyroGain*alpha; // gyro gain
    
    Eigen::Quaterniond omegaBarQuat;
    omegaBarQuat.x() = omegaBar(0);
    omegaBarQuat.y() = omegaBar(1);
    omegaBarQuat.z() = omegaBar(2);
    omegaBarQuat.w() = 0;

    Eigen::Quaterniond quatDot = (quat * omegaBarQuat);
    Eigen::Vector3d biasDot = -cfParams.biasGain * alpha; // bias gain

    propogate_quaternion(dt, quatDot);
    gyroBias = gyroBias + biasDot * dt;

    quat.normalize();

    return true;

}

void CompFilter::propogate_quaternion(double dt, const Eigen::Quaterniond& quatDot)
{
    quat.x() = quat.x() + quatDot.x()*dt/2;
    quat.y() = quat.y() + quatDot.y()*dt/2;
    quat.z() = quat.z() + quatDot.z()*dt/2;
    quat.w() = quat.w() + quatDot.w()*dt/2;
}

Eigen::Vector3d CompFilter::vee(const Eigen::Matrix3d& m)
{
    Eigen::Vector3d vector;
    vector << m(2,1) , m(0,2), m(1,0);

    return vector;
}

Eigen::Quaterniond CompFilter::get_quat() const
{
    return quat;
}

Eigen::Vector3d CompFilter::get_bias() const
{
    return gyroBias;
}

}  // namespace cf
