cmake_minimum_required(VERSION 3.20)
project(roslean_twist_mux LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roslean_core REQUIRED)

add_executable(twist_mux src/twist_mux_node.cpp)
target_link_libraries(twist_mux
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  roslean_core::roslean_core)
# The Lean libraries live outside the install tree, so bake their location in.
set_target_properties(twist_mux PROPERTIES
  BUILD_RPATH "${ROSLEAN_RPATH}"
  INSTALL_RPATH "${ROSLEAN_RPATH}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS twist_mux DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # End-to-end scenarios over real topics under simulated time. If upstream
  # twist_mux is installed, also require identical decisions from it.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(twist_mux QUIET)
  set(_scenario_args "$<TARGET_FILE:twist_mux>")
  if(twist_mux_FOUND)
    list(APPEND _scenario_args "twist_mux/twist_mux")
  else()
    message(WARNING "upstream twist_mux not found: scenarios run without the comparison")
  endif()
  add_test(NAME scenario
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/scenario.py" ${_scenario_args})
  # A domain of its own, so that other ROS traffic on the machine cannot interfere.
  set_tests_properties(scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=87" TIMEOUT 600)
endif()

ament_package()
