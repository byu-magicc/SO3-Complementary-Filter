import sqlite3

from yaml import parse
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import argparse
import os, sys

import numpy as np
from scipy.spatial.transform import Rotation
import math

import matplotlib.pyplot as plt
from boat_inekf.ros2bag_plotter_tools import plot_position, plot_rotation

class BagFileParser():
    """
    A simple parser for ROS2 bags using sqlite. This parser will convert the rosbag into a .txt file format that can 
    then be used with the estimator in this package. Created for use with ROS Foxy which lacks rosbag2_py.

    Bag should contain the following:
        Estimate: position, velocity, and orientation (nav_msgs/msg/Odometry)
        MOCAP position/orientation measurements (geometry_msgs/msg/PoseStamped)
    
    WARNING: Ensure your workspace has sourced ONLY your ROS2 disto. Otherwise this parser may confuse message types 
    with the ROS1 equivalents
    """
    def __init__(self, bagfile):
        self.conn = sqlite3.connect(bagfile)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

def main(bagfile, outfile=None, plot_estimate=True, offline=False, end_t=np.inf):

  outpath, outpathMocap, outpathEstimate = generate_outpaths(bagfile, outfile)

  # if (os.path.exists(outpathMocap) and os.path.exists(outpathMocap)):
  #   print("Already parsed. Proceeding to plot.")
  # else:
  parse(bagfile, outpath, outpathEstimate, outpathMocap)
    
  plot(outpathMocap, outpathEstimate, plot_estimate, offline, end_t)

def parse(bagfile, outpath, outpathEstimate, outpathMocap):
  parser = BagFileParser(bagfile)

  parse_estimate(parser, outpathEstimate, bagfile)
  parse_mocap(parser, outpathMocap)

  
  print(f"Data written to {outpath}")


def plot(outpathMocap, outpathEstimate, plot_estimate, offline, end_t):

  try:
    mocap_data = np.loadtxt(outpathMocap, dtype=float)
    estimate_data = np.loadtxt(outpathEstimate, dtype=float)
    t_start = mocap_data[0,0]

    mocap_data[:,0] -= t_start
    estimate_data[:,0] -= t_start
    mocap_t = mocap_data[:,0]
    estimate_t = estimate_data[:,0]
    
    # Cut off end time
    if end_t != np.inf:
      mocap_idx = mocap_t < end_t
      est_idx = estimate_t < end_t
      mocap_data = mocap_data[mocap_idx]
      estimate_data = estimate_data[est_idx]

    mocap_t = mocap_data[:,0]
    estimate_t = estimate_data[:,0]

    # Resolve time offset when estimation is done on a different computer
    if offline:
      offset = estimate_t[0] - mocap_t[0]
      estimate_t -= offset

    estimate_vel_x = estimate_data[:,4]
    estimate_vel_y = estimate_data[:,5]
    estimate_vel_z = estimate_data[:,6]

    plot_position(mocap_t, mocap_data, estimate_t, estimate_data, plot_estimate)
    plot_rotation(mocap_t, mocap_data, estimate_t, estimate_data, plot_estimate)

    plt.show(block=False)
    plt.pause(0.001) # Pause for interval seconds.
    input("hit [enter] to end.")
    plt.close('all') # all open plots are correctly closed after each run

  except FileNotFoundError as e:
    print(e)


def generate_outpaths(bagfile, outfile):
  # Write bags to .txt
  if not outfile:
      outpath = os.path.join("/", *bagfile.split('/')[:-1], "out")
  else:
      assert outfile.split('.')[-1] == 'txt', f"Provided path must end with a .txt file, got: {outfile}"
      outpath = os.path.join(*outfile.split('.')[:-1])

  outpathMocap = outpath + "_mocap.txt"
  outpathEstimate = outpath + "_estimate.txt"

  return outpath, outpathMocap, outpathEstimate

def parse_estimate(parser, outpathEstimate, bagfile):
  estimate = parser.get_messages('/estimate')

  with open(outpathEstimate, 'w') as f:
    if '/estimator_' in bagfile: # if this substring is in the bagfile name, then the program recognizes that it needs to calculate a time offset
                                 # (b/c the estimator was run separately on a playing bag file, not in conjunction with all the sensors in real time)
      timeOffset = get_time_offset(parser, estimate) # lines up mocap data with estimate data on timeline
    else:
      timeOffset = 0.0
    print("Time Offset: ", timeOffset)

    for msg in estimate:
      stamp = msg[1].header.stamp
      time = stamp.sec + stamp.nanosec * 1E-9 - timeOffset
      pos = msg[1].pose.pose.position
      quaternion = msg[1].pose.pose.orientation
      velocity = msg[1].twist.twist.linear

      axisAngle = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_rotvec()

      if (axisAngle[2] < -np.pi/2.0):
        axisAngle[2]+=np.pi*2

      f.write("{:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f}".format(time, pos.x, pos.y, pos.z, velocity.x, velocity.y, velocity.z, axisAngle[0], axisAngle[1], axisAngle[2]))
      f.write("\n")

def get_time_offset(parser, estimate):
  for msg in estimate:
    pos = msg[1].pose.pose.position
    stamp = msg[1].header.stamp
    timeRaw = stamp.sec + stamp.nanosec * 1E-9 # get unchanged time stamp from estimate
    if (pos.x != 0): # this finds the time when apriltag started publishing
      apriltag = parser.get_messages('/apriltag_pose')
      apriltagStartTime = apriltag[0][1].header.stamp.sec + apriltag[0][1].header.stamp.nanosec *1E-9
      return timeRaw - apriltagStartTime
  
  return 0.0

def parse_mocap(parser, outpathMocap):
  mocap_data = parser.get_messages('/rhodey/pose_ned')
    
  with open(outpathMocap, 'w') as f:

    for msg in mocap_data:
      stamp = msg[1].header.stamp
      time = stamp.sec + stamp.nanosec * 1E-9
      pos = msg[1].pose.position
      quaternion = msg[1].pose.orientation

      axisAngle = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_rotvec()

      if (axisAngle[2] < -np.pi/2.0):
        axisAngle[2]+=np.pi*2
      

      f.write("{:f} {:f} {:f} {:f} {:f} {:f} {:f}".format(time, pos.x, pos.y, pos.z, axisAngle[0], axisAngle[1], axisAngle[2]))
      f.write("\n")


if __name__ == "__main__":
    # Parse through arguments
    parser = argparse.ArgumentParser(description="Convert a ROS2 rosbag to .txt file for calibation")
    parser.add_argument("-b", "--bagfile", default= '/home/boat-landing/data/timeStampTests/mocap/timeStampTest/mocap_01-19-2023_11:48:07_0.db3',type=str, help="ROS2 bagfile (*.db3) to read data from.")
    parser.add_argument("-o", "--outfile" ,type=str, help="Output path including a .txt file that will be used as a prefix (e.g. out.txt -> out_imu.txt etc.). Uses the original bagfile directory if none provided.")
    parser.add_argument("-p", "--plot_estimate", type=bool, default = True)
    parser.add_argument("-f", "--offline", action='store_true', help="Set if estimation is done at a different time from the data")
    parser.add_argument("-e", "--end_t", type=float, default = np.inf)
    args = vars(parser.parse_args())
    main(**args)
