cmake_minimum_required(VERSION 3.8)
project(meta_chassis_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(omni_chassis_controller_parameters
  src/omni_chassis_controller.yaml
)
generate_parameter_library(agv_chassis_controller_parameters
  src/agv_chassis_controller.yaml
)
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/omni_chassis_controller.cpp
  src/omni_wheel_kinematics.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(meta_chassis_controller
  omni_chassis_controller_parameters
  agv_chassis_controller_parameters)

pluginlib_export_plugin_description_file(
  controller_interface meta_chassis_controller.xml)

if(BUILD_TESTING)

endif()

ament_auto_package()
