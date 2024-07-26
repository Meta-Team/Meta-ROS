cmake_minimum_required(VERSION 3.8)
project(omni_wheel_controller)

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

# Add omni_wheel_controller library related compile commands
generate_parameter_library(omni_wheel_controller_parameters
  src/omni_wheel_controller.yaml
  include/omni_wheel_controller/validate_omni_wheel_controller_parameters.hpp
)
add_library(
  omni_wheel_controller
  SHARED
  src/omni_wheel_controller.cpp
  src/omni_wheel_kinematics.cpp
)
target_include_directories(omni_wheel_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(omni_wheel_controller omni_wheel_controller_parameters)
ament_target_dependencies(omni_wheel_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(omni_wheel_controller PRIVATE "OMNI_WHEEL_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface omni_wheel_controller.xml)

install(
  TARGETS
  omni_wheel_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)

  # ament_add_gmock(test_load_omni_wheel_controller test/test_load_omni_wheel_controller.cpp)
  # target_include_directories(test_load_omni_wheel_controller PRIVATE include)
  # ament_target_dependencies(
  #   test_load_omni_wheel_controller
  #   controller_manager
  #   hardware_interface
  #   ros2_control_test_assets
  # )

  # add_rostest_with_parameters_gmock(test_omni_wheel_controller test/test_omni_wheel_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/omni_wheel_controller_params.yaml)
  # target_include_directories(test_omni_wheel_controller PRIVATE include)
  # target_link_libraries(test_omni_wheel_controller omni_wheel_controller)
  # ament_target_dependencies(
  #   test_omni_wheel_controller
  #   controller_interface
  #   hardware_interface
  # )

  # add_rostest_with_parameters_gmock(test_omni_wheel_controller_preceeding test/test_omni_wheel_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/omni_wheel_controller_preceeding_params.yaml)
  # target_include_directories(test_omni_wheel_controller_preceeding PRIVATE include)
  # target_link_libraries(test_omni_wheel_controller_preceeding omni_wheel_controller)
  # ament_target_dependencies(
  #   test_omni_wheel_controller_preceeding
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
  omni_wheel_controller
)

ament_package()
