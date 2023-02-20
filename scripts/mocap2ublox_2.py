#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from boat_inekf.pose2ublox_ros_2 import pose2ublox_ros_2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy

class Mocap2Ublox_2(pose2ublox_ros_2):
    """Class that listens for mocap NED information and stores it for synthetic measurment generation"""
    def __init__(self):
        super().__init__()
        qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        qos_mocap = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        self.receivedRoverPose = False
        self.receivedBasePose = False
        # Subscribers
        self.rover_mocap_ned_sub_ = self.create_subscription(PoseStamped, 'rover_mocap', self.roverNedCallback, qos_mocap)
        self.base_mocap_ned_sub_ = self.create_subscription(PoseStamped, 'base_mocap', self.baseNedCallback, qos_mocap)
        # Publisher
        self.truth_pub_ = self.create_publisher(Odometry, '/relPosTruth', qos_profile=qos)
        self.truth = Odometry()

    def roverNedCallback(self, msg):
        self.p2u.rover_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])
        self.publishTruth(msg.header.stamp)
        if not self.receivedPose:
            self.receivedRoverPose = True
            if self.receivedBasePose:
                self.receivedPose = True

    def baseNedCallback(self, msg):

        self.p2u.base_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])

        self.p2u.compass_quat = np.array([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        
        if not self.receivedPose:
            self.receivedBasePose = True
            if self.receivedRoverPose:
                self.receivedPose = True

    def publishTruth(self, stamp):
        relpos_array = self.p2u.rover_ned - self.p2u.base_ned
        self.truth.header.stamp = stamp
        self.truth.pose.pose.position.x = relpos_array[0]
        self.truth.pose.pose.position.y = relpos_array[1]
        self.truth.pose.pose.position.z = relpos_array[2]

        self.truth_pub_.publish(self.truth)

def main(args=None): 
    rclpy.init(args=args)
    mocap_2_ublox = Mocap2Ublox_2()
    rclpy.spin(mocap_2_ublox)
    mocap_2_ublox.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
