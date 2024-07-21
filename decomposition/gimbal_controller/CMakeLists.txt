cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  controller_interface
  control_msgs
  control_toolbox
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  sensor_msgs
  geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add gimbal_controller library related compile commands
generate_parameter_library(gimbal_controller_parameters
  src/gimbal_controller.yaml
  include/gimbal_controller/validate_gimbal_controller_parameters.hpp
)
add_library(
  gimbal_controller
  SHARED
  src/gimbal_controller.cpp
)
target_include_directories(gimbal_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(gimbal_controller gimbal_controller_parameters)
ament_target_dependencies(gimbal_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(gimbal_controller PRIVATE "gimbal_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface gimbal_controller.xml)

install(
  TARGETS
  gimbal_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)

  # ament_add_gmock(test_load_gimbal_controller test/test_load_gimbal_controller.cpp)
  # target_include_directories(test_load_gimbal_controller PRIVATE include)
  # ament_target_dependencies(
  #   test_load_gimbal_controller
  #   controller_manager
  #   hardware_interface
  #   ros2_control_test_assets
  # )

  # add_rostest_with_parameters_gmock(test_gimbal_controller test/test_gimbal_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/gimbal_controller_params.yaml)
  # target_include_directories(test_gimbal_controller PRIVATE include)
  # target_link_libraries(test_gimbal_controller gimbal_controller)
  # ament_target_dependencies(
  #   test_gimbal_controller
  #   controller_interface
  #   hardware_interface
  # )

  # add_rostest_with_parameters_gmock(test_gimbal_controller_preceeding test/test_gimbal_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/gimbal_controller_preceeding_params.yaml)
  # target_include_directories(test_gimbal_controller_preceeding PRIVATE include)
  # target_link_libraries(test_gimbal_controller_preceeding gimbal_controller)
  # ament_target_dependencies(
  #   test_gimbal_controller_preceeding
  #   controller_interface
  #   hardware_interface
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  gimbal_controller
)

ament_package()
