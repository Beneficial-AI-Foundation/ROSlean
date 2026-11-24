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

# The tests and their configuration, so that they can be run from the install
# tree against any node:
#   python3 <prefix>/share/roslean_collision_monitor/test/scenario.py <package>/<executable>
install(DIRECTORY test DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE)

# --- Gazebo demo (docs/TUTORIAL_COLLISION_MONITOR.md) -------------------------
# Launch file, monitor configuration, obstacle scripts and narrator. The
# automated check, demo/gazebo/check_demo.py, is not a colcon test: it needs
# the simulator with a display and the downloaded furniture (`pixi run cm-check`).
install(DIRECTORY demo DESTINATION share/${PROJECT_NAME})
# --- end of Gazebo demo -------------------------------------------------------

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
  set_tests_properties(smoke_node PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=94" TIMEOUT 300)

  # Black-box tests under simulated time; what they pin down is described in
  # docs/collision_monitor/UPSTREAM_BEHAVIOUR.md. The node under test is the
  # first argument of each script. To run a node of this package through the
  # same tests, add a test with "$<TARGET_FILE:the_node>" in place of
  # ${_upstream} (and --no-tf if it uses no transforms, as the reference
  # node below). Lifecycle nodes are detected and brought to the active state.
  find_package(nav2_collision_monitor QUIET)
  set(_test "${CMAKE_CURRENT_SOURCE_DIR}/test")
  set(_upstream "nav2_collision_monitor/collision_monitor")
  set(_reference "${_test}/reference_node.py")

  # The Python model as a plain node: checks the harness on a node that is
  # not upstream's and the model against the fixed scenarios. Upstream
  # aborts in negative-stamp; the model does not.
  add_test(NAME reference_scenario
    COMMAND "${Python3_EXECUTABLE}" "${_test}/scenario.py" "${_reference}"
      --no-tf --skip negative-stamp)
  # Domains of their own, so that other ROS traffic on the machine cannot interfere.
  # LABELS long: a test that takes more than about a minute. `pixi run
  # test-quick` leaves these out; `pixi run test` runs everything.
  set_tests_properties(reference_scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=95" TIMEOUT 1800
    LABELS long)

  if(nav2_collision_monitor_FOUND)
    # Fixed scenarios with the outputs observed from upstream.
    add_test(NAME upstream_scenario
      COMMAND "${Python3_EXECUTABLE}" "${_test}/scenario.py" ${_upstream})
    set_tests_properties(upstream_scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=89" TIMEOUT 3600
      LABELS long)
    # Random configurations and event sequences: upstream against itself
    # (which measures nondeterminism) and against the Python model.
    add_test(NAME upstream_random_diff
      COMMAND "${Python3_EXECUTABLE}" "${_test}/random_diff.py" ${_upstream} ${_upstream} 0 20)
    set_tests_properties(upstream_random_diff PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=90" TIMEOUT 2400
      LABELS long)
    # The same with two different implementations side by side.
    add_test(NAME reference_random_diff
      COMMAND "${Python3_EXECUTABLE}" "${_test}/random_diff.py" "${_reference}" ${_upstream} 0 5
        --no-tf)
    set_tests_properties(reference_random_diff PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=96" TIMEOUT 1200
      LABELS long)
  else()
    message(WARNING "nav2_collision_monitor not found: the tests against upstream are not registered")
  endif()
endif()

ament_package()
