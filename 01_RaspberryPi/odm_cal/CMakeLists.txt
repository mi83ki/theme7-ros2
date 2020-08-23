cmake_minimum_required(VERSION 2.8.3)
project(odm_cal)
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs
)

add_executable(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/${PROJECT_NAME}/${PROJECT_NAME}.h
  #include/${PROJECT_NAME}/${PROJECT_NAME}.h
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
##  ${OpenCV_LIBRARIES}
  ptr_msgs_generate_messages_cpp
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
