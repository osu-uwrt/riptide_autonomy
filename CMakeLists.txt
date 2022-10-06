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
      "Stderr shown above. Stdout shown below: \n" #stderr automatically printed by cmake, stdout has to be manually printed
      "${func_output}\n")
    message(FATAL_ERROR "Review error information above. Cmake will exit.") #will quit cmake execution
  endif()
endfunction()

# generate headers...
execute_process_with_error_detection(
  ./src/riptide_autonomy/assistant/btassistant.py generate_headers ${PROJECT_BINARY_DIR}/autonomy_generator/include
)

# ...and behaviortree node registrator
execute_process_with_error_detection(
  ./src/riptide_autonomy/assistant/btassistant.py generate_registrator ${PROJECT_BINARY_DIR}/autonomy_generator/registrator.cpp
)

#configure includes
include_directories(${PROJECT_BINARY_DIR}/autonomy_generator/include)
include_directories(${PROJECT_SOURCE_DIR}/include)

#find all c++ source files to compile (all of them, not just the special classes)
file(GLOB bt_actions_sources RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/bt_actions/*.cpp)
file(GLOB bt_conditions_sources RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/bt_conditions/*.cpp)
file(GLOB bt_decorators_sources RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/bt_decorators/*.cpp)

#
# autonomy library
#
add_library(autonomy SHARED
  #utilities
  src/riptide_autonomy/util.cpp

  #bt nodes
  ${bt_actions_sources}
  ${bt_conditions_sources}
  ${bt_decorators_sources}

  #registrator
  ${PROJECT_BINARY_DIR}/autonomy_generator/registrator.cpp
)

target_compile_definitions(autonomy PRIVATE BT_PLUGIN_EXPORT)

# autonomy lib deps
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
  robot_localization
)

# Tree execution executable
add_executable(doTask src/riptide_autonomy/DoTask.cpp)

# set the name of the autonomy package
target_compile_definitions(doTask PUBLIC AUTONOMY_PKG_NAME="${PROJECT_NAME}")

ament_target_dependencies(doTask
  ament_index_cpp
  rclcpp
  behaviortree_cpp_v3
  rclcpp_action
  riptide_msgs2
)

#install library and node
install(TARGETS
  doTask autonomy
  DESTINATION lib/${PROJECT_NAME})

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
