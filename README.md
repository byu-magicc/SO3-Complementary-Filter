# SO3 Complementary Filter
A ROS2 package that wraps a complementary filter for estimating Pitch and Roll on the Lie group SO3 of an IMU with gyro and accel measurements. Based on the filter introduced [here](https://ieeexplore.ieee.org/document/4608934) and explained nicely in the context of multirotors [here](https://ieeexplore.ieee.org/document/6289431).

The project is built as a ros2 package with `colcon build --symlink-install` 

After building the filter can be run with `ros2 launch so3_cf estimator_launch.py`

Filter gains are configured in `params.yaml`