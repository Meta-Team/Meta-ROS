cmake_minimum_required(VERSION 3.8)
project(super_capacitor)

set (CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# find_package(pluginlib REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp_components)

add_library(super_capacitor SHARED
  src/super_capacitor_controller.cpp
  src/xidi_capacitor_driver.cpp
  src/pacific_spirit_capacitor_driver.cpp
)

target_include_directories(super_capacitor PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(super_capacitor
  PLUGIN "SuperCapacitorController"
  EXECUTABLE super_capacitor_node)

ament_target_dependencies(super_capacitor
  rclcpp
  rclcpp_components
  serial_driver
  device_interface
  pluginlib
  meta_hardware
)

pluginlib_export_plugin_description_file(super_capacitor plugins.xml)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS super_capacitor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  super_capacitor
)
ament_export_targets(
  export_${PROJECT_NAME}
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
