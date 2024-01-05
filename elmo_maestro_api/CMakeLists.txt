cmake_minimum_required(VERSION 3.8)
project(elmo_maestro_api)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS "-O3 -Wall -fPIC -no-pie -pthread")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads)

add_library(libMMC_APP SHARED IMPORTED)
set_target_properties(libMMC_APP PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMMC_APP_LinuxRpc64.a)
add_library(libMMCPP SHARED IMPORTED)
set_target_properties(libMMCPP PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMMCPP_LinuxRpc64.a)

add_executable(motor_test src/motor_test.cpp)
target_include_directories(motor_test PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/elmo_maestro_api
  ${CMAKE_CURRENT_SOURCE_DIR}/include/MMC_APP/Src/OsCommon/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/include/MMC_APP/Src/OsPlatform/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/include/MMCPP/Src/OsCommon/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/include/MMCPP/Src/OsPlatform/includes)

target_link_libraries(motor_test
  pthread
  libMMC_APP
  libMMCPP)

install(TARGETS motor_test
  DESTINATION lib/${PROJECT_NAME})

# add_executable(sdk_test src/sdk_test.cpp)
# target_include_directories(sdk_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(sdk_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS sdk_test
#   DESTINATION lib/${PROJECT_NAME})

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
