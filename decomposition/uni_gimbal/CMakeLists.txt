cmake_minimum_required(VERSION 3.8)
project(uni_gimbal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(aiming_interface REQUIRED)
find_package(movement_interface REQUIRED)
find_package(motor_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(uni_gimbal_node 
  src/uni_gimbal.cpp
  src/gimbal.cpp)

ament_target_dependencies(uni_gimbal_node
  rclcpp
  aiming_interface
  movement_interface
  geometry_msgs
  motor_interface)

install(TARGETS uni_gimbal_node
  DESTINATION lib/${PROJECT_NAME})

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
