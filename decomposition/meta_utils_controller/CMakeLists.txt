cmake_minimum_required(VERSION 3.8)  # the mininal version
project(meta_utils_controller)  # define the project name here, and it influence the variable ${PROJECT_NAME}

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
  armor_tester_controller_parameters
  src/armor_tester_controller.yaml
)

# generate_parameter_lib (
#     name_of_target, -> to generate target_name.cpp
#     yaml_file, -> generate the parameter listeners based on which yaml file
# )

ament_auto_add_library(
  armor_tester_controller
  SHARED
  src/armor_tester_controller.cpp
)

# ament_auto_add_library (
#     target_package_name
#     SHARED/STATIC/OBJECT
#     sorce file
# )

target_include_directories(
  armor_tester_controller
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
  armor_tester_controller
  armor_tester_controller_parameters
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
