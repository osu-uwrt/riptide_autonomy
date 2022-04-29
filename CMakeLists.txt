cmake_minimum_required(VERSION 3.8)
project(riptide_autonomy2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories("include")
include_directories($(rclcpp_INCLUDE_DIRS))

#
# autonomy library
# PUT YOUR STATE FILES HERE!!
#
#
add_library(autonomy STATIC
  src/riptide_autonomy/util.cpp
  src/riptide_autonomy/SimpleStates.cpp
  src/riptide_autonomy/SimpleConditions.cpp
  src/riptide_autonomy/UWRTSyncActionNode.cpp
  src/riptide_autonomy/RetryUntilSuccessfulOrTimeout.cpp
  src/riptide_autonomy/BigMoveState.cpp
  src/riptide_autonomy/FlattenCalculationState.cpp
  src/riptide_autonomy/LineDriveCalcState.cpp
  src/riptide_autonomy/SearchState.cpp
  src/riptide_autonomy/ToWorldFrameState.cpp
  src/riptide_autonomy/VelocityState.cpp
  src/riptide_autonomy/ActuateState.cpp
  src/riptide_autonomy/GetActuatorStatus.cpp
  src/riptide_autonomy/ActuatorStateCheckers.cpp
  src/riptide_autonomy/WaitState.cpp
  src/riptide_autonomy/AlignTorpedoState.cpp
)

#deps
ament_target_dependencies(autonomy 
  rclcpp
  geometry_msgs 
  std_msgs 
  nav_msgs 
  sensor_msgs
  riptide_msgs2 
  behaviortree_cpp_v3 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

#add and link doTask executable
add_executable(doTask src/riptide_autonomy/DoTask.cpp)
target_link_libraries(doTask autonomy ${colcon_LIBRARIES})

#install C++ stuff
install(TARGETS
  doTask
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/riptide_autonomy/actions/AlignTorpedoAction.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  src/riptide_autonomy/actions/utils
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
