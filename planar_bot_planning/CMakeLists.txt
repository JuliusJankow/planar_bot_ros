cmake_minimum_required(VERSION 2.8.3)
project(planar_bot_planning)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  planar_bot_hw
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

# add node for prm generation
add_executable(create_prm_node src/create_prm_node.cpp src/prm.cpp)

target_link_libraries(create_prm_node
  ${catkin_LIBRARIES}
)

# add node for prm
add_executable(prm_node src/prm_node.cpp src/prm.cpp src/elastic_band.cpp src/topp.cpp)

target_link_libraries(prm_node
  ${catkin_LIBRARIES}
)
