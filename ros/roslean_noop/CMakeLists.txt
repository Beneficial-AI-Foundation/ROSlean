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
# The Lean libraries live outside the install tree, so bake their location in.
set_target_properties(noop_node PROPERTIES
  BUILD_RPATH "${ROSLEAN_RPATH}"
  INSTALL_RPATH "${ROSLEAN_RPATH}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS noop_node DESTINATION lib/${PROJECT_NAME})

ament_package()
