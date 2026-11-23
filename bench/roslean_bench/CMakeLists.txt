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

set_target_properties(bench_core load_probe PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../../roslean_core/lib;$ORIGIN/..")

install(TARGETS bench_core load_probe DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/run_bench.py DESTINATION lib/${PROJECT_NAME})

ament_package()
