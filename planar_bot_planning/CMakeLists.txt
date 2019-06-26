cmake_minimum_required(VERSION 2.8.3)
project(planar_bot_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Eigen3 REQUIRED)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# add node for prm
add_executable(prm_node src/prm_node.cpp src/prm.cpp)

# add node for smoothing and topp
