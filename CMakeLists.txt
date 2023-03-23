cmake_minimum_required(VERSION 3.5)
project(so3_cf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ublox_read_2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

install(DIRECTORY
  launch params 
  DESTINATION share/${PROJECT_NAME})


add_library(${PROJECT_NAME} SHARED
    src/estimator/compfilter.cpp 
)

target_include_directories(${PROJECT_NAME} PUBLIC include/estimator ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include/estimator)

add_executable(cf_ros 
  src/ros/cf_node.cpp 
  src/ros/cf_ros.cpp 
  src/ros/cf_ros_callback.cpp 
  src/ros/cf_ros_init.cpp 
  src/ros/utils.cpp)
  
ament_target_dependencies(cf_ros rclcpp sensor_msgs geometry_msgs ublox_read_2 Eigen3)

target_link_libraries(cf_ros so3_cf)
target_include_directories(cf_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  cf_ros
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  so3_cf
  DESTINATION lib/
)

ament_package()
