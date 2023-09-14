cmake_minimum_required(VERSION 3.0.2)
project(planar_2dof_unit5)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  gazebo_plugins
  rospy
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# force plugin
add_library(force SHARED src/force.cpp)
target_link_libraries(force ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(

)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)