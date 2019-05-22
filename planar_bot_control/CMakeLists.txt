cmake_minimum_required(VERSION 3.0)

set (CMAKE_CXX_STANDARD 11)

project(planar_bot_control)

find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  control_toolbox
  forward_command_controller
  realtime_tools
  urdf
)

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    control_toolbox
    forward_command_controller
    realtime_tools
    urdf
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/rmrc_controller.cpp
  src/asc_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
