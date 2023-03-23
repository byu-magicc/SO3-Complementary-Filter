#include "compfilter.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <math.h>

namespace cf {

CompFilter::CompFilter(CompFilterParams params) 
    : cfParams{params}, gravity{9.81}, getInitBias{true}, gyroAvgCount{0}, compassReceived{false}
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
    
    // std::cout << "\nIMU\n" << gyro << "\n" << accel;
    Eigen::Matrix3d R = quat.toRotationMatrix();
    Eigen::Vector3d Rz = R.transpose().block<3,1>(0,2);
    Eigen::Matrix3d alphaM = cfParams.accelGain/(gravity*gravity) * (accel * Rz.transpose() - Rz * accel.transpose()); //accel gain

    if (compassReceived)
    {
        // assumes that the compass measurement is a full R with current pitch/roll and incoming yaw
        // auto euler = R.eulerAngles(2,1,0);
        Eigen::Vector3d euler = quat_to_euler();
        double roll = euler(0);
        double pitch = euler(1);
        double yaw = euler(2);
        // std::cout << "\nEULER: \n" << euler*180/3.14159 << "\n\n" << R.transpose().eulerAngles(2,1,0)*180/3.14159;
        // std::cout << "\nQUAT: \n" << quat.w() << "\n" << quat.vec() << "\n\n" << R;
        // std::cout << "\nR: \n" << R << "\n\n" << euler_to_rotation(pitch, roll, yaw);

        Eigen::Quaterniond qCompass;
        qCompass = Eigen::AngleAxisd(compass, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        
        Eigen::Matrix3d RCompass = qCompass.toRotationMatrix();
        // std::cout << "\nR: \n" << R << "\n\n" << RCompass;
        // Eigen::Matrix3d RCompass = euler_to_rotation(pitch, roll, yaw);
        
        Eigen::Matrix3d compassError = (R*RCompass.transpose() - RCompass*R.transpose())/2;
        compassError(0,2) = 0.0;
        compassError(1,2) = 0.0;
        compassError(2,0) = 0.0;
        compassError(2,1) = 0.0;
        // std::cout << "\n\nCompass\n" << compass << "\n" << compassError;
        alphaM +=  cfParams.compassGain * compassError; 
        
        compassReceived = false;
    }

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

void CompFilter::compass_update(double zCompass)
{
    compass = zCompass;
    compassReceived = true;
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

Eigen::Vector3d CompFilter::quat_to_euler()
{    
    double q0 = quat.w();
    double qx = quat.x();
    double qy = quat.y();
    double qz = quat.z();
    double phi = atan2(2.0 * (qy * qz + q0 * qx), q0*q0 - qx*qx - qy*qy + qz*qz );
    double theta = asin(2.0 * (q0 * qy - qx * qz));
    double psi = atan2(2.0 * (qx * qy + q0 * qz), q0*q0 + qx*qx - qy*qy - qz*qz);

    Eigen::Vector3d euler = (Eigen::Vector3d() << phi, theta, psi).finished();
    
    return euler;
}

Eigen::Matrix3d CompFilter::euler_to_rotation(double phi, double theta, double psi)
{

    double c_phi = cos(phi);
    double s_phi = sin(phi);
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_psi = cos(psi);
    double s_psi = sin(psi);

    Eigen::Matrix3d R_roll;
    Eigen::Matrix3d R_pitch;
    Eigen::Matrix3d R_yaw;

    R_roll << 1, 0, 0,
            0, c_phi, -s_phi,
            0, s_phi, c_phi;
    R_pitch << c_theta, 0, s_theta,
            0, 1, 0,
            -s_theta, 0, c_theta;
    R_yaw << c_psi, -s_psi, 0,
            s_psi, c_psi, 0,
            0, 0, 1;
    Eigen::Matrix3d R = R_yaw * R_pitch * R_roll;
    
    return R;

}
}  // namespace cf
