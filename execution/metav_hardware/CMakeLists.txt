cmake_minimum_required(VERSION 3.8)
project(metav_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(angles REQUIRED)

if (NOT TARGET sockcanpp)
    add_subdirectory(libsockcanpp)
endif()

add_library(
  metav_hardware
  SHARED
  src/hardware_interface.cpp
  src/motor_network/dji_motor_network.cpp
  src/motor_driver/dji_motor_driver.cpp
  src/motor_network/mi_motor_network.cpp
  src/motor_driver/mi_motor_driver.cpp
)
target_include_directories(
  metav_hardware
  PUBLIC
  include
)
target_link_libraries(
  metav_hardware
  sockcanpp
)
ament_target_dependencies(
  metav_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  angles
)
# prevent pluginlib from using boost
target_compile_definitions(metav_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface metav_hardware.xml)

install(
  TARGETS
  metav_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  # find_package(ament_cmake_gmock REQUIRED)
  # find_package(ros2_control_test_assets REQUIRED)

  # ament_add_gmock(test_hardware_interface test/test_hardware_interface.cpp)
  # target_include_directories(test_hardware_interface PRIVATE include)
  # ament_target_dependencies(
  #   test_hardware_interface
  #   hardware_interface
  #   pluginlib
  #   ros2_control_test_assets
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  metav_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
