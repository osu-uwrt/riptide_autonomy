cmake_minimum_required(VERSION 3.8)
project(riptide_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories("include")
include_directories($(rclcpp_INCLUDE_DIRS))

#
#
# PUT YOUR STATE FILES HERE!!
#
#
add_library(autonomy STATIC
  src/riptide_autonomy/util.cpp
  src/riptide_autonomy/UWRTSyncActionNode.cpp
  src/riptide_autonomy/BigMoveState.cpp
  src/riptide_autonomy/FlattenCalculationState.cpp
  src/riptide_autonomy/LineDriveCalcState.cpp
  src/riptide_autonomy/SearchState.cpp
  src/riptide_autonomy/ToWorldFrameState.cpp
  src/riptide_autonomy/VelocityState.cpp
)

ament_target_dependencies(autonomy 
  rclcpp
  geometry_msgs 
  std_msgs 
  nav_msgs 
  sensor_msgs 
  behaviortree_cpp_v3 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

add_executable(doTask src/riptide_autonomy/DoTask.cpp)
add_executable(transformTest src/riptide_autonomy/testscripts/doTransformTest.cpp)
add_executable(frameCaster src/riptide_autonomy/testscripts/testFrameBroadcaster.cpp)
target_link_libraries(doTask autonomy ${colcon_LIBRARIES})
target_link_libraries(transformTest autonomy ${colcon_LIBRARIES})
target_link_libraries(frameCaster autonomy ${colcon_LIBRARIES})

install(TARGETS
  doTask
  transformTest
  frameCaster
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
