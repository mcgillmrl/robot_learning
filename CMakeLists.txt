cmake_minimum_required(VERSION 2.8.3)
project(robot_learning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aqua_gait
  aquacore
  roscpp
  tf
  roslib
  std_srvs
  gazebo_msgs
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules/)
find_package(Eigen3 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

add_message_files(
  FILES
  ExperienceData.msg
)

add_service_files(
  FILES
  T2VInfo.srv
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_learning
  CATKIN_DEPENDS roscpp roslib tf std_srvs message_runtime gazebo_msgs
#  DEPENDS system_lib
)