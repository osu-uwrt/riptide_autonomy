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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)

#sm specific depends
find_package(chameleon_tf_msgs REQUIRED)
find_package(nortek_dvl_msgs REQUIRED)


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
  vision_msgs
  riptide_msgs2
  robot_localization
  std_srvs
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tinyxml2_vendor
)

#
# util library
#
add_library(autonomy_utils SHARED
  src/riptide_autonomy/util.cpp
  src/riptide_autonomy/ROSEnabledNode.cpp
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
add_library(autonomy_conditions SHARED
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
add_library(autonomy_decorators SHARED
  ${PROJECT_BINARY_DIR}/autonomy_generator/registerDecorators.cpp
)

target_compile_definitions(autonomy_decorators PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(autonomy_decorators PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonomy_decorators ${deps})
target_link_libraries(autonomy_decorators autonomy_utils)

#
# Health
#

# library
file(GLOB health_sources RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/health/*.cpp)
add_library(autonomy_health
  ${health_sources})

target_include_directories(autonomy_health PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(autonomy_health autonomy_utils)

# bt checker executable
add_executable(check_bt src/riptide_autonomy/check_bt.cpp)
target_link_libraries(check_bt autonomy_utils autonomy_health)


#
# Tree execution executable
#
add_executable(autonomy src/riptide_autonomy/autonomy.cpp)

target_include_directories(autonomy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(autonomy autonomy_utils)
target_compile_definitions(autonomy PUBLIC AUTONOMY_PKG_NAME="${PROJECT_NAME}")

ament_target_dependencies(autonomy
  ament_index_cpp
  rclcpp
  behaviortree_cpp
  rclcpp_action
  riptide_msgs2
)

#install executables
install(TARGETS autonomy check_bt
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
file(GLOB assistant_py_files RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_autonomy/assistant/*.py)
install(PROGRAMS
  ${assistant_py_files}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/riptide_autonomy/headless_interface.py
  DESTINATION lib/${PROJECT_NAME})

#install bt assistant "actions" directory
install(DIRECTORY src/riptide_autonomy/assistant/actions
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME})

if(AUTONOMY_BUILD_TESTS)
  #set up GTest
  find_package(ament_cmake_gtest QUIET)

  if(ament_cmake_gtest_FOUND)
    #ament lint auto things
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    #
    # node tester
    #

    #glob all test sources together
    file(GLOB action_test_sources RELATIVE ${PROJECT_SOURCE_DIR} test/riptide_autonomy/bt_actions/*.cpp)
    file(GLOB condition_test_sources RELATIVE ${PROJECT_SOURCE_DIR} test/riptide_autonomy/bt_conditions/*.cpp)
    file(GLOB decorator_test_sources RELATIVE ${PROJECT_SOURCE_DIR} test/riptide_autonomy/bt_decorators/*.cpp)
    file(GLOB tool_test_sources RELATIVE ${PROJECT_SOURCE_DIR} test/riptide_autonomy/tools/*.cpp)

    ament_add_gtest(test_nodes
      ${action_test_sources}
      ${condition_test_sources}
      ${decorator_test_sources}
      ${tool_test_sources}
      test/riptide_autonomy/DummyActionNode.cpp
      test/riptide_autonomy/BtTestTool.cpp
      test/riptide_autonomy/BtTest.cpp
      test/riptide_autonomy/executeTests.cpp #overrides default main() method
    )

    ament_target_dependencies(test_nodes ${deps})
    target_link_libraries(test_nodes autonomy_utils)

    target_include_directories(test_nodes PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    
    install(TARGETS test_nodes
      DESTINATION lib/${PROJECT_NAME})

    #
    # health utilities tester
    #
    file(GLOB health_test_sources RELATIVE ${PROJECT_SOURCE_DIR} test/riptide_autonomy/health/*.cpp)
    ament_add_gtest(test_health_utils
      ${health_test_sources})

    target_link_libraries(test_health_utils autonomy_health)

    target_include_directories(test_health_utils PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

    #executable
    install(TARGETS test_health_utils
      DESTINATION lib/${PROJECT_NAME})
    
    #test trees
    install(DIRECTORY test/riptide_autonomy/health/health_test_trees
      DESTINATION share/${PROJECT_NAME})
  endif()
endif()

ament_package()
