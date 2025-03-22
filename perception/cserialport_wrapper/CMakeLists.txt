cmake_minimum_required(VERSION 3.8)
project(cserialport_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CSerialPortRootPath "${CMAKE_CURRENT_SOURCE_DIR}/CSerialPort")
list(APPEND CSerialPortSourceFiles ${CSerialPortRootPath}/src/SerialPort.cpp ${CSerialPortRootPath}/src/SerialPortBase.cpp ${CSerialPortRootPath}/src/SerialPortInfo.cpp ${CSerialPortRootPath}/src/SerialPortInfoBase.cpp)
list(APPEND CSerialPortSourceFiles ${CSerialPortRootPath}/src/SerialPortInfoUnixBase.cpp ${CSerialPortRootPath}/src/SerialPortUnixBase.cpp)

ament_auto_add_library( ${PROJECT_NAME} SHARED ${CSerialPortSourceFiles})
target_include_directories(${PROJECT_NAME} PUBLIC ${CSerialPortRootPath}/include)

# remove prefix
# set_target_properties( ${PROJECT_NAME} PROPERTIES PREFIX "")

# preprocessor definitions for compiling CSerialPort library
# set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS BUILDING_LIBCSERIALPORT)

ament_export_include_directories("${CSerialPortRootPath}/include")
install(
  DIRECTORY ${CSerialPortRootPath}/include
  DESTINATION include
)

target_link_libraries( ${PROJECT_NAME} pthread)

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
