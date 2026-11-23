cmake_minimum_required(VERSION 3.20)
project(roslean_bench LANGUAGES C CXX)

# Benchmarks only. Not under ros/, so the normal build and the Docker image
# do not include it: it downloads upstream's sources at configure time.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(roslean_core REQUIRED)

# Upstream's own sources, at the version of the binary that the node-level
# benchmark runs (ros-jazzy-twist-mux in pixi.toml). The in-process benchmark
# compiles them unchanged and calls TwistMux::hasPriorityStamped and the
# handle callbacks directly. Set UPSTREAM_TWIST_MUX_DIR to use a checkout.
set(UPSTREAM_TWIST_MUX_TAG "4.5.0" CACHE STRING "Tag of ros-teleop/twist_mux to compare against")
if(NOT UPSTREAM_TWIST_MUX_DIR)
  include(FetchContent)
  FetchContent_Declare(upstream_twist_mux
    GIT_REPOSITORY https://github.com/ros-teleop/twist_mux.git
    GIT_TAG "${UPSTREAM_TWIST_MUX_TAG}"
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR does_not_exist)  # fetch only; do not add its CMakeLists
  FetchContent_MakeAvailable(upstream_twist_mux)
  set(UPSTREAM_TWIST_MUX_DIR "${upstream_twist_mux_SOURCE_DIR}")
endif()

add_library(upstream_twist_mux_objs STATIC
  "${UPSTREAM_TWIST_MUX_DIR}/src/twist_mux.cpp"
  "${UPSTREAM_TWIST_MUX_DIR}/src/twist_mux_diagnostics.cpp")
target_include_directories(upstream_twist_mux_objs SYSTEM PUBLIC "${UPSTREAM_TWIST_MUX_DIR}/include")
target_compile_options(upstream_twist_mux_objs PRIVATE -w)
target_link_libraries(upstream_twist_mux_objs PUBLIC
  rclcpp::rclcpp diagnostic_updater::diagnostic_updater
  ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${visualization_msgs_TARGETS})

add_executable(bench_core src/bench_core.cpp)
target_compile_options(bench_core PRIVATE -Wall -Wextra)
target_link_libraries(bench_core upstream_twist_mux_objs roslean_core::roslean_core)

add_executable(load_probe src/load_probe.cpp)
target_compile_options(load_probe PRIVATE -Wall -Wextra)
target_link_libraries(load_probe rclcpp::rclcpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

# Collision monitor (docs/collision_monitor/COMPARISON.md). bench_cm calls
# the callbacks of upstream's own library (nav2_collision_monitor, from the
# environment) and the callback bodies of the ROSlean node in one process;
# cm_load drives running nodes over topics.
find_package(sensor_msgs REQUIRED)
find_package(nav2_collision_monitor QUIET)
set(_cm_targets cm_load)
add_executable(cm_load src/cm_load.cpp)
target_compile_options(cm_load PRIVATE -Wall -Wextra)
target_link_libraries(cm_load rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})
if(APPLE)
  # The message targets include the Python generators' libraries, which need
  # the interpreter's symbols and abort a plain executable when it starts.
  target_link_options(cm_load PRIVATE -Wl,-dead_strip_dylibs)
endif()
if(nav2_collision_monitor_FOUND)
  add_executable(bench_cm src/bench_cm.cpp)
  # As the node's own build: no fused multiply-add in the conversions.
  target_compile_options(bench_cm PRIVATE -Wall -Wextra -ffp-contract=off)
  target_include_directories(bench_cm PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../ros/roslean_collision_monitor/include")
  ament_target_dependencies(bench_cm nav2_collision_monitor rclcpp sensor_msgs geometry_msgs)
  target_link_libraries(bench_cm roslean_core::roslean_core)
  list(APPEND _cm_targets bench_cm)
else()
  message(WARNING "nav2_collision_monitor not found: bench_cm is not built")
endif()

if(APPLE)
  set(_origin "@loader_path")
else()
  set(_origin "$ORIGIN")
endif()
set_target_properties(bench_core load_probe ${_cm_targets} PROPERTIES
  INSTALL_RPATH "${_origin}/../../../roslean_core/lib;${_origin}/..")

install(TARGETS bench_core load_probe ${_cm_targets} DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/run_bench.py scripts/run_cm_bench.py DESTINATION lib/${PROJECT_NAME})

ament_package()
