cmake_minimum_required(VERSION 3.8)  # the mininal version
project(armor_tester)  # define the project name here, and it influence the variable ${PROJECT_NAME}

# only acts for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)  # ament_cmake is mandatory
find_package(ament_cmake_auto REQUIRED)  # ament_cmake_auto is also mandatory
ament_auto_find_build_dependencies()  # scan the package.xml to get the dependencies

# generate parameter library for controller
generate_parameter_library(
  ${PROJECT_NAME}_controller_parameters
  src/armor_tester_controller.yaml
)

# generate_parameter_lib (
#     name_of_target, -> to generate target_name.cpp
#     yaml_file, -> generate the parameter listeners based on which yaml file
# )

# construct controller shared library to complile the .so lib
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/armor_tester.cpp
)

ament_auto_add_library(
  ${PROJECT_NAME}_controller
  SHARED
  src/armor_tester_controller.cpp
)

# ament_auto_add_library (
#     target_package_name
#     SHARED/STATIC/OBJECT
#     sorce file
# )

# register a class in library as plugin, here, the ArmorTester
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ArmorTester"
  EXECUTABLE ${PROJECT_NAME}_node
)
# rclcpp_components_register_node(
#     <library_target>  # existing library name
#     PLUGIN "<class_name>"  # existing class name
#     EXECUTABLE <executable_name>  # self-designed
# )

# add the include file
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(
  ${PROJECT_NAME}_controller
  PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# target_include_directories(
#     cmake-target
#     PUBLIC
#     <BUILD_INTERFACE:.....> # used during build
#     <INSTALL_INTERFACE:...> # used during install
# )

# link the the parameter listeners
target_link_libraries(
  ${PROJECT_NAME}_controller
  ${PROJECT_NAME}_controller_parameters
)
# target_link_libraries(
#     cmake-target-package
#     cmake-target that cmake-target-package depends on
# )

# where to find the description of the controller
pluginlib_export_plugin_description_file(
  controller_interface
  armor_tester_controller.xml
)
# pluginlib_export_plugin_description_file(
#     base-class
#     description file
# )

# test for building
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
