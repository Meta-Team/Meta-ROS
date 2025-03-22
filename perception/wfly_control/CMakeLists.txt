cmake_minimum_required(VERSION 3.8)
project(wfly_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(wfly_control SHARED
  src/wfly_control.cpp
  src/wfly_sbus.cpp)

target_include_directories(wfly_control PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(wfly_control
  PLUGIN "WflyControl"
  EXECUTABLE wfly_control_node)

ament_target_dependencies(wfly_control
  rclcpp
  rclcpp_components
  operation_interface
  cserialport_wrapper
)

install(TARGETS
  wfly_control
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS wfly_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
