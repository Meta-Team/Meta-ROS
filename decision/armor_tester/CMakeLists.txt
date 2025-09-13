cmake_minimum_required(VERSION 3.8)
project(armor_tester)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  ${PROJECT_NAME}_parameters
  src/armor_tester_controller.yaml
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/armor_tester.cpp
  src/armor_tester_controller.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_parameters
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ArmorTester"
  EXECUTABLE ${PROJECT_NAME}_node
  )

pluginlib_export_plugin_description_file(
  controller_interface armor_tester_controller.xml)


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

ament_auto_package()
