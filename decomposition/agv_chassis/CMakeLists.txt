cmake_minimum_required(VERSION 3.8)
project(agv_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(device_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behavior_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(agv_chassis
  src/agv_chassis.cpp
  src/agv_kinematics.cpp)

ament_target_dependencies(agv_chassis
  rclcpp
  device_interface
  behavior_interface
  geometry_msgs
  )

install(TARGETS
  agv_chassis
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
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
