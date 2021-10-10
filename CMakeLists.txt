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
  src/riptide_autonomy/velocity_state.cpp
  src/riptide_autonomy/search_state.cpp
  src/riptide_autonomy/torpedo_align_state.cpp
)

add_executable(doTask src/riptide_autonomy/do_task.cpp)
target_link_libraries(doTask states ${catkin_LIBRARIES})
