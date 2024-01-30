cmake_minimum_required(VERSION 3.8)
project(elmo_maestro_hardware)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14) # Need C++14 for lib to compile
endif()

set(CMAKE_CXX_FLAGS "-O3 -Wall -fPIC -no-pie -pthread")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(elmo_maestro_api REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(motor_test src/motor_test.cpp)
target_include_directories(motor_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motor_test elmo_maestro_api)

## Setup target
add_library(
  ${PROJECT_NAME}
  SHARED
  src/elmo_maestro_hi.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
set(DEPENDENCIES
  elmo_maestro_api
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

pluginlib_export_plugin_description_file(hardware_interface elmo_maestro_hi_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )

install(TARGETS motor_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
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
