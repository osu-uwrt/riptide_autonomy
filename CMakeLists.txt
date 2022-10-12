cmake_minimum_required(VERSION 3.8)
project(riptide_autonomy2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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
find_package(robot_localization REQUIRED)
find_package(ament_index_cpp REQUIRED)

#create directory for auto-generated code which will include headers for custom nodes as well as source code that registers custom nodes
make_directory(${PROJECT_BINARY_DIR}/autonomy_generator)

#
# cmake macro to execute a process and quit cmake execution if it fails
#
function(execute_process_with_error_detection)
  execute_process(COMMAND ${ARGV}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE func_out
    RESULT_VARIABLE func_result
  )

  if(${func_result}) #nonzero function result means error
    message(SEND_ERROR 
      "Error occurred during execution of process: ${cmd}\n\n"
      "Stderr shown above. Stdout shown below: \n" #stderr automatically printed by cmake/colcon, stdout has to be manually printed
      "${func_output}\n")
    message(FATAL_ERROR "Review error information above. Cmake will exit.") #will quit cmake execution
  endif()
endfunction()

# generate behaviortree node registrators
execute_process_with_error_detection(
  ./src/riptide_autonomy/assistant/btassistant.py generate_registrators ${PROJECT_BINARY_DIR}/autonomy_generator
)

set(deps 
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
  robot_localization
)

#
# util library
#
add_library(autonomy_utils SHARED
  src/riptide_autonomy/util.cpp
)

target_include_directories(autonomy_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonomy_utils ${deps})

#
# action library
#
file(GLOB simple_actions_sources RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/bt_actions/*.simple.cpp)
add_library(autonomy_actions SHARED
  ${simple_actions_sources}
  ${PROJECT_BINARY_DIR}/autonomy_generator/registerActions.cpp
)

target_include_directories(autonomy_actions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonomy_actions ${deps})
target_link_libraries(autonomy_actions autonomy_utils)

target_compile_definitions(autonomy_actions PRIVATE BT_PLUGIN_EXPORT)


#
# condition library
#
file(GLOB simple_conditions_sources RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/bt_conditions/*.simple.cpp)
add_library(autonomy_conditions SHARED
  ${simple_conditions_sources}
  ${PROJECT_BINARY_DIR}/autonomy_generator/registerConditions.cpp
)

target_compile_definitions(autonomy_conditions PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(autonomy_conditions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonomy_conditions ${deps})
target_link_libraries(autonomy_conditions autonomy_utils)


#
# decorator library
#
file(GLOB simple_decorators_sources RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/bt_decorators/*.simple.cpp)
add_library(autonomy_decorators SHARED
  ${simple_decorators_sources}
  ${PROJECT_BINARY_DIR}/autonomy_generator/registerDecorators.cpp
)

target_compile_definitions(autonomy_decorators PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(autonomy_decorators PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonomy_decorators ${deps})
target_link_libraries(autonomy_decorators autonomy_utils)


#
# Tree execution executable
#
add_executable(doTask src/riptide_autonomy/DoTask.cpp)

target_include_directories(doTask PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# set the name of the autonomy package
target_compile_definitions(doTask PUBLIC AUTONOMY_PKG_NAME="${PROJECT_NAME}")

ament_target_dependencies(doTask
  ament_index_cpp
  rclcpp
  behaviortree_cpp_v3
  rclcpp_action
  riptide_msgs2
)

#install executable
install(TARGETS doTask
  DESTINATION lib/${PROJECT_NAME}
)

#install libraries
install(TARGETS autonomy_actions autonomy_conditions autonomy_decorators autonomy_utils
  DESTINATION lib
)

# install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
file(GLOB ros_actions_py RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/ros_actions/*.py)
install(PROGRAMS
  ${ros_actions_py}
  src/riptide_autonomy/assistant/btassistant.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  src/riptide_autonomy/ros_actions/utils
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
