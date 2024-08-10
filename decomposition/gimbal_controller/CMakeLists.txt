cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add gimbal_controller library related compile commands
generate_parameter_library(gimbal_controller_parameters
  src/gimbal_controller.yaml
  include/gimbal_controller/validate_gimbal_controller_parameters.hpp
)
ament_auto_add_library(
  gimbal_controller
  SHARED
  src/gimbal_controller.cpp
)
target_include_directories(gimbal_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(gimbal_controller gimbal_controller_parameters)
target_compile_definitions(gimbal_controller PRIVATE "gimbal_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface gimbal_controller.xml)

if(BUILD_TESTING)

endif()

ament_auto_package()
