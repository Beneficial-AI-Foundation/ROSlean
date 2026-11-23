cmake_minimum_required(VERSION 3.20)
project(roslean_collision_monitor LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -ffp-contract=off: the conversions are written operation by operation
  # (include/roslean_collision_monitor/conversions.hpp); the compiler must not
  # fuse a multiplication and an addition into one rounding.
  add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(roslean_core REQUIRED)

add_executable(collision_monitor src/collision_monitor_node.cpp)
target_include_directories(collision_monitor PRIVATE include)
target_link_libraries(collision_monitor
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  roslean_core::roslean_core)
# Find libroslean_core relative to the executable, so that the install tree
# works wherever it is copied (isolated and merged colcon layouts).
if(APPLE)
  set(_origin "@loader_path")
else()
  set(_origin "$ORIGIN")
endif()
set_target_properties(collision_monitor PROPERTIES
  INSTALL_RPATH "${_origin}/../../../roslean_core/lib;${_origin}/..")

install(TARGETS collision_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # The trusted floating-point work of the shim together with the core's
  # conversion to millimetres, without ROS.
  add_executable(smoke_conversions test/smoke_conversions.cpp)
  target_include_directories(smoke_conversions PRIVATE include)
  target_link_libraries(smoke_conversions roslean_core::roslean_core)
  add_test(NAME smoke_conversions COMMAND smoke_conversions)

  # The core's timeouts against rclcpp::Duration, which upstream uses.
  add_executable(smoke_timeouts test/smoke_timeouts.cpp)
  target_link_libraries(smoke_timeouts rclcpp::rclcpp roslean_core::roslean_core)
  add_test(NAME smoke_timeouts COMMAND smoke_timeouts)

  # The node over real topics under simulated time: pass-through, slowdown,
  # stop, and the configurations it must refuse.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME smoke_node
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_node.py"
      "$<TARGET_FILE:collision_monitor>")
  # A domain of its own, so that other ROS traffic on the machine cannot interfere.
  set_tests_properties(smoke_node PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=89" TIMEOUT 300)
endif()

ament_package()
