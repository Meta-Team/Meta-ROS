cmake_minimum_required(VERSION 3.8)
project(unitree_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(Python3_FIND_STRATEGY "VERSION")
set(Python3_FIND_VERSION 3.8)
find_package(Python3 COMPONENTS Development Interpreter)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motor_interface REQUIRED)

include_directories(include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
  set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
  set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

add_executable(unitree_controller src/unitree_controller.cpp src/unitree_driver.cpp)
target_link_libraries(unitree_controller ${EXTRA_LIBS})
ament_target_dependencies(unitree_controller
  rclcpp
  motor_interface)

set_target_properties(unitree_controller PROPERTIES INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(LIBRARY_OUTPUT_PATH "{CMAKE_CURRENT_SOURCE_DIR}/lib")
add_subdirectory(thirdparty/pybind11)
pybind11_add_module(unitree_actuator_sdk thirdparty/python_wrapper/wrapper.cpp)
target_link_libraries(unitree_actuator_sdk PRIVATE ${EXTRA_LIBS})
set_target_properties(unitree_actuator_sdk PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")

# install(
#   FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Linux64.so
#   DESTINATION lib/${PROJECT_NAME}
# )
install(TARGETS unitree_controller
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY lib/
  DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS unitree_controller
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

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
