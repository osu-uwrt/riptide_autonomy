cmake_minimum_required(VERSION 3.5)

project(riptide_autonomy)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  geometry_msgs
  tf2_geometry_msgs
  sensor_msgs
  behaviortree_cpp_v3
  moveit_ros_planning_interface
  tf2
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    behaviortree_cpp_v3)

include_directories("include")
include_directories(${catkin_INCLUDE_DIRS})

add_library(states STATIC
  src/riptide_autonomy/big_move_state.cpp
  src/riptide_autonomy/flatten_calculation_state.cpp
  src/riptide_autonomy/to_world_frame_state.cpp
)

add_executable(move src/riptide_autonomy/move.cpp)
add_executable(flatten src/riptide_autonomy/flatten.cpp)
add_executable(gate src/riptide_autonomy/gate_task.cpp)
target_link_libraries(move states ${catkin_LIBRARIES})
target_link_libraries(flatten states ${catkin_LIBRARIES})
target_link_libraries(gate states ${catkin_LIBRARIES})
