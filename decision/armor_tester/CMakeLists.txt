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

# construct controller shared library to complile the .so lib
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/armor_tester.cpp
)

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
