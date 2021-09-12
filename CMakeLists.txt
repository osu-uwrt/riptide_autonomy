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
  sensor_msgs
  behaviortree_cpp_v3
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    behaviortree_cpp_v3)

# target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3)

