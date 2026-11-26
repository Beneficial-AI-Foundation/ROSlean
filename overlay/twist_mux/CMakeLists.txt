cmake_minimum_required(VERSION 3.20)
project(twist_mux LANGUAGES C CXX)

# The Lean-backed node of ROSlean under the package name of upstream
# twist_mux, so that a workspace that contains this package shadows upstream's
# and `ros2 run twist_mux twist_mux`, `ros2 launch twist_mux
# twist_mux_launch.py` and launch files that name `package='twist_mux'` start
# the Lean-backed node without being edited. See docs/DROP_IN_TWIST_MUX.md.
#
# Nothing is implemented here. The executable is compiled from the source of
# ros/roslean_twist_mux, and the config files are the ones of that package.
# This package is outside `ros/` on purpose: the tests of roslean_twist_mux
# compare with whatever package is called twist_mux, which must be upstream's.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

get_filename_component(ROSLEAN_TWIST_MUX_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../ros/roslean_twist_mux" REALPATH)
if(NOT EXISTS "${ROSLEAN_TWIST_MUX_DIR}/src/twist_mux_node.cpp")
  message(FATAL_ERROR
    "${ROSLEAN_TWIST_MUX_DIR}/src/twist_mux_node.cpp not found: this package "
    "builds only inside a checkout of the ROSlean repository")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roslean_core REQUIRED)

add_executable(twist_mux "${ROSLEAN_TWIST_MUX_DIR}/src/twist_mux_node.cpp")
target_link_libraries(twist_mux
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  roslean_core::roslean_core)
# As in roslean_twist_mux: find libroslean_core relative to the executable
# (isolated and merged colcon layouts).
if(APPLE)
  set(_origin "@loader_path")
else()
  set(_origin "$ORIGIN")
endif()
set_target_properties(twist_mux PROPERTIES
  INSTALL_RPATH "${_origin}/../../../roslean_core/lib;${_origin}/..")

install(TARGETS twist_mux DESTINATION lib/${PROJECT_NAME})
# The helper nodes are upstream's; these names forward to them.
install(PROGRAMS scripts/twist_marker scripts/joystick_relay.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# The same files, under the same names, as upstream installs.
install(FILES
  "${ROSLEAN_TWIST_MUX_DIR}/config/twist_mux_locks.yaml"
  "${ROSLEAN_TWIST_MUX_DIR}/config/twist_mux_topics.yaml"
  "${ROSLEAN_TWIST_MUX_DIR}/config/joystick.yaml"
  DESTINATION share/${PROJECT_NAME}/config)

# Lets other packages tell this package from upstream after find_package.
ament_package(CONFIG_EXTRAS cmake/twist_mux-extras.cmake)
