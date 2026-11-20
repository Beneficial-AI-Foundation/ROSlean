cmake_minimum_required(VERSION 3.20)
project(roslean_noop LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roslean_core REQUIRED)

add_executable(noop_node src/noop_node.cpp)
target_link_libraries(noop_node
  rclcpp::rclcpp
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
set_target_properties(noop_node PROPERTIES
  INSTALL_RPATH "${_origin}/../../../roslean_core/lib;${_origin}/..")

install(TARGETS noop_node DESTINATION lib/${PROJECT_NAME})

ament_package()
