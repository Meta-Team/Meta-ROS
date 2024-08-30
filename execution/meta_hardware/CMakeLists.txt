cmake_minimum_required(VERSION 3.8)
project(meta_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(angles REQUIRED)
find_package(Boost REQUIRED)
find_package(serial_driver REQUIRED)

add_library(
  meta_hardware
  SHARED
  src/dji_motor_interface.cpp
  src/motor_network/dji_motor_network.cpp
  src/motor_driver/dji_motor_driver.cpp
  src/mi_motor_interface.cpp
  src/motor_network/mi_motor_network.cpp
  src/motor_driver/mi_motor_driver.cpp
  src/dm_motor_interface.cpp
  src/motor_network/dm_motor_network.cpp
  src/motor_driver/dm_motor_driver.cpp
  src/brt_encoder_interface.cpp
  src/motor_network/brt_encoder_network.cpp
  src/motor_driver/brt_encoder_driver.cpp
  src/im1253c_interface.cpp
  src/modbus_rtu_driver/modbus_rtu_driver.cpp
)
target_include_directories(
  meta_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  meta_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  angles
  Boost
  serial_driver
)
# prevent pluginlib from using boost
target_compile_definitions(meta_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface meta_hardware.xml)

install(
  TARGETS
  meta_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  meta_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
