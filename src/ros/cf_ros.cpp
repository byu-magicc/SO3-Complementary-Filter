#include <cf_ros.h>

namespace cf
{

CF_ROS::CF_ROS() : Node("cf_ros") {

   init_params();
   instantiate_estimator();
   init_subs_and_pubs();
   // publish_estimate();
   
}

void CF_ROS::publish_estimate() const {
   geometry_msgs::msg::QuaternionStamped msg;

   msg.header.stamp = this->now();
   msg.quaternion = get_estimator_quaternion();

   attitudePub_->publish(msg);

   sensor_msgs::msg::Imu biasMsg;
   biasMsg.header.stamp = msg.header.stamp;
   biasMsg.angular_velocity = get_estimator_bias();

   gyroBiasPub_->publish(biasMsg);
}

}
