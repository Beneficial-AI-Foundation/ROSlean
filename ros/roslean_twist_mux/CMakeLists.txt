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
# Find libroslean_core relative to the executable, so that the install tree
# works wherever it is copied, with or without the environment set up
# (isolated and merged colcon layouts).
if(APPLE)
  set(_origin "@loader_path")
else()
  set(_origin "$ORIGIN")
endif()
set_target_properties(twist_mux PROPERTIES
  INSTALL_RPATH "${_origin}/../../../roslean_core/lib;${_origin}/..")

install(TARGETS twist_mux DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config demo DESTINATION share/${PROJECT_NAME})

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
  if(twist_mux_FOUND)
    # Random configurations and event sequences, both nodes against a model.
    add_test(NAME random_diff
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/random_diff.py"
        "$<TARGET_FILE:twist_mux>" "twist_mux/twist_mux" 0 20)
    set_tests_properties(random_diff PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=88" TIMEOUT 2400)
  endif()
endif()

ament_package()
