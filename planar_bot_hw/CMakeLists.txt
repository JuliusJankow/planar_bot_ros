cmake_minimum_required(VERSION 2.8.3)
project(planar_bot_hw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planar_bot_hw
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/planar_bot_hw_node.cpp src/planar_bot_hw.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
