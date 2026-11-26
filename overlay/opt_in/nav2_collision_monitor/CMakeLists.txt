cmake_minimum_required(VERSION 3.20)
project(nav2_collision_monitor LANGUAGES C CXX)

# The Lean-backed collision monitor of ROSlean under the package name of
# upstream nav2_collision_monitor, so that a workspace that contains this
# package shadows upstream's, and `ros2 run nav2_collision_monitor
# collision_monitor`, `ros2 launch nav2_collision_monitor
# collision_monitor_node.launch.py` and launch files that name
# `package='nav2_collision_monitor'` (nav2_bringup's navigation_launch.py)
# start the Lean-backed node without being edited.
# See docs/DROP_IN_COLLISION_MONITOR.md, and read its first section before
# using this: the node provides stop and slowdown zones on one laser scan
# and refuses every other configuration, Nav2's default included.
#
# Nothing is implemented here. The executable and the component are compiled
# from the source of ros/roslean_collision_monitor, and the params file is the
# one of that package. This package is outside `ros/` on purpose: the tests of
# roslean_collision_monitor compare with whatever package is called
# nav2_collision_monitor, which must be upstream's.
#
# colcon does not find this package by itself: the directory above has a
# COLCON_IGNORE file, so that cloning the repository into a workspace does not
# replace Nav2's collision monitor. It is built by naming its directory:
#   colcon build --base-paths src src/ROSlean/overlay/opt_in/nav2_collision_monitor \
#     --packages-up-to nav2_collision_monitor
#
# What upstream's package has and this one does not: the libraries
# collision_monitor_core and collision_detector_core as CMake exports, and the
# headers include/nav2_collision_monitor/*.hpp. The collision detector is
# upstream's, reached through this package (see the end of this file).

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # The flags of roslean_collision_monitor. -ffp-contract=off is part of the
  # node's arithmetic (see that package's CMakeLists.txt), not a preference.
  add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off)
endif()
# As in roslean_collision_monitor: without a build type the node would be
# compiled without optimisation, and the points of a scan would then differ
# from upstream's on macOS arm64.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "nav2_collision_monitor (ROSlean overlay): no build type given, using Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

get_filename_component(ROSLEAN_CM_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../ros/roslean_collision_monitor" REALPATH)
if(NOT EXISTS "${ROSLEAN_CM_DIR}/src/collision_monitor_node.cpp")
  message(FATAL_ERROR
    "${ROSLEAN_CM_DIR}/src/collision_monitor_node.cpp not found: this package "
    "builds only inside a checkout of the ROSlean repository")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(roslean_core REQUIRED)

set(_node_libraries
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_util::nav2_util_core
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${rosgraph_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  roslean_core::roslean_core)

add_executable(collision_monitor "${ROSLEAN_CM_DIR}/src/collision_monitor_node.cpp")
target_include_directories(collision_monitor PRIVATE "${ROSLEAN_CM_DIR}/include")
target_link_libraries(collision_monitor ${_node_libraries})

# The component, under upstream's plugin name (src/collision_monitor_component.cpp).
# The library is not called collision_monitor_core, as upstream's is, so that
# it can never be loaded in the place of upstream's by a program that links
# against that one.
add_library(roslean_collision_monitor_overlay SHARED src/collision_monitor_component.cpp)
target_include_directories(roslean_collision_monitor_overlay PRIVATE
  "${ROSLEAN_CM_DIR}/include" "${ROSLEAN_CM_DIR}/src")
target_link_libraries(roslean_collision_monitor_overlay ${_node_libraries}
  rclcpp_components::component)

# As in roslean_collision_monitor: find libroslean_core relative to the
# binaries (isolated and merged colcon layouts).
if(APPLE)
  set(_origin "@loader_path")
else()
  set(_origin "$ORIGIN")
endif()
set(_rpath_last "")
if(APPLE)
  # Last resort on macOS, for the case that this package is installed into
  # another install base than roslean_core (`pixi run build-overlay` does
  # that): the directory in which libroslean_core is now. On Linux the
  # library is then found through LD_LIBRARY_PATH; macOS removes
  # DYLD_LIBRARY_PATH from the environment of a process started through a
  # system binary such as /bin/bash.
  set(_rpath_last ";$<TARGET_FILE_DIR:roslean_core::roslean_core>")
endif()
set_target_properties(collision_monitor PROPERTIES
  INSTALL_RPATH "${_origin}/../../../roslean_core/lib;${_origin}/..${_rpath_last}")
set_target_properties(roslean_collision_monitor_overlay PROPERTIES
  INSTALL_RPATH "${_origin}/../../roslean_core/lib;${_origin}${_rpath_last}")

install(TARGETS collision_monitor DESTINATION lib/${PROJECT_NAME})
install(TARGETS roslean_collision_monitor_overlay
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
# The collision detector is upstream's; this name forwards to it.
install(PROGRAMS scripts/collision_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# The same file, under the same name, as upstream installs.
install(FILES "${ROSLEAN_CM_DIR}/params/collision_monitor_params.yaml"
  DESTINATION share/${PROJECT_NAME}/params)
# Lets the tests of roslean_collision_monitor tell this package from upstream.
install(FILES ROSLEAN_OVERLAY DESTINATION share/${PROJECT_NAME})

# The entry of this package in the ament index of components, which is what a
# component container reads for `package='nav2_collision_monitor'`. Written
# here and not by rclcpp_components_register_nodes because of its second line.
set(_components
  "nav2_collision_monitor::CollisionMonitor;lib/$<TARGET_FILE_NAME:roslean_collision_monitor_overlay>\n")
# The detector's component is upstream's library. This package hides
# upstream's entry in the index, so the library is named here by its full
# path, as it is found now. If upstream is not installed, or is moved later,
# `nav2_collision_monitor::CollisionDetector` cannot be loaded through this
# package.
set(_upstream_lib_dirs "")
foreach(_prefix $ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" _prefix "${_prefix}")
  foreach(_p ${_prefix})
    if(NOT "${_p}" STREQUAL "${CMAKE_INSTALL_PREFIX}")
      list(APPEND _upstream_lib_dirs "${_p}/lib")
    endif()
  endforeach()
endforeach()
find_library(ROSLEAN_UPSTREAM_COLLISION_DETECTOR_LIBRARY collision_detector_core
  PATHS ${_upstream_lib_dirs} NO_DEFAULT_PATH)
if(ROSLEAN_UPSTREAM_COLLISION_DETECTOR_LIBRARY)
  message(STATUS "nav2_collision_monitor (ROSlean overlay): the component "
    "nav2_collision_monitor::CollisionDetector is upstream's, "
    "${ROSLEAN_UPSTREAM_COLLISION_DETECTOR_LIBRARY}")
  string(APPEND _components
    "nav2_collision_monitor::CollisionDetector;${ROSLEAN_UPSTREAM_COLLISION_DETECTOR_LIBRARY}\n")
else()
  message(WARNING "upstream nav2_collision_monitor was not found on AMENT_PREFIX_PATH: "
    "the component nav2_collision_monitor::CollisionDetector will not be loadable "
    "while this package is sourced")
endif()
ament_index_register_resource("rclcpp_components" CONTENT "${_components}")

# Lets other packages tell this package from upstream after find_package.
ament_package(CONFIG_EXTRAS cmake/nav2_collision_monitor-extras.cmake)
