cmake_minimum_required(VERSION 2.8.3)
project(com_micon)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
 geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES com_micon
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosmarica_keyboard src/rosmarica_keyboard.cpp)
target_link_libraries(rosmarica_keyboard ${catkin_LIBRARIES})
add_dependencies(rosmarica_keyboard ${catkin_EXPORTED_TARGETS})

