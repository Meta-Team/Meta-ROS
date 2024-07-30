cmake_minimum_required(VERSION 3.8)
project(meta_chassis_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_toolbox
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  Eigen3
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add omni_chassis_controller library related compile commands
generate_parameter_library(omni_chassis_controller_parameters
  src/omni_chassis_controller.yaml
  include/meta_chassis_controller/validate_omni_chassis_controller_parameters.hpp
)
add_library(
  meta_chassis_controller
  SHARED
  src/omni_chassis_controller.cpp
  src/omni_wheel_kinematics.cpp
)
target_include_directories(meta_chassis_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(meta_chassis_controller omni_chassis_controller_parameters)
ament_target_dependencies(meta_chassis_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  controller_interface meta_chassis_controller.xml)

install(
  TARGETS
  meta_chassis_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  omni_chassis_controller
)

ament_package()
