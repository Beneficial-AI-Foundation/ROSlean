cmake_minimum_required(VERSION 3.20)
project(roslean_collision_monitor LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -ffp-contract=off: the conversions are written operation by operation
  # (include/roslean_collision_monitor/conversions.hpp), and so is tf2's
  # inline arithmetic that transforms.hpp applies to every point; the compiler
  # must not fuse a multiplication and an addition into one rounding. On every
  # platform (decided by the user, tasks/PLAN.md).
  add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(roslean_core REQUIRED)
# The Nav2 extension (SPEC_NAV2.md, 8.5): lifecycle node with bond, the state
# message, transforms.
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
# What the tests of the trusted headers need (transforms.hpp calls tf2).
# roslean_core is in the list also where it is not called: on macOS its
# link options drop the message libraries' Python typesupport, which would
# otherwise abort a C++ program at load time (see ros/roslean_core).
set(_transform_libraries
  tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs ${geometry_msgs_TARGETS}
  roslean_core::roslean_core)

add_executable(collision_monitor src/collision_monitor_node.cpp)
target_include_directories(collision_monitor PRIVATE include)
target_link_libraries(collision_monitor ${_node_libraries})

# The same node as an rclcpp component (SPEC_NAV2.md, 8.5, ND10): plugin
# roslean_collision_monitor::CollisionMonitorNode.
add_library(collision_monitor_component SHARED src/collision_monitor_node.cpp)
target_compile_definitions(collision_monitor_component PRIVATE ROSLEAN_COLLISION_MONITOR_COMPONENT)
target_include_directories(collision_monitor_component PRIVATE include)
target_link_libraries(collision_monitor_component ${_node_libraries}
  rclcpp_components::component)
rclcpp_components_register_nodes(collision_monitor_component
  "roslean_collision_monitor::CollisionMonitorNode")
# Find libroslean_core relative to the executable, so that the install tree
# works wherever it is copied (isolated and merged colcon layouts).
if(APPLE)
  set(_origin "@loader_path")
else()
  set(_origin "$ORIGIN")
endif()
set_target_properties(collision_monitor PROPERTIES
  INSTALL_RPATH "${_origin}/../../../roslean_core/lib;${_origin}/..")
set_target_properties(collision_monitor_component PROPERTIES
  INSTALL_RPATH "${_origin}/../../roslean_core/lib;${_origin}")

install(TARGETS collision_monitor DESTINATION lib/${PROJECT_NAME})
install(TARGETS collision_monitor_component
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
# launch/collision_monitor_node.launch.py and params/collision_monitor_params.yaml
# have the names and the place of upstream's (docs/DROP_IN_COLLISION_MONITOR.md).
install(DIRECTORY config launch params DESTINATION share/${PROJECT_NAME})

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
  target_link_libraries(smoke_conversions roslean_core::roslean_core ${_transform_libraries})
  add_test(NAME smoke_conversions COMMAND smoke_conversions)

  # The trusted code that looks transforms up and applies them (T2, T6):
  # which transform is asked for, every kind of failure, and the arithmetic
  # of N25 bit for bit against a reference written out operation by
  # operation. Fails if the package is compiled without -ffp-contract=off on
  # a machine with fused multiply-add.
  add_executable(smoke_transforms test/smoke_transforms.cpp)
  target_include_directories(smoke_transforms PRIVATE include)
  target_link_libraries(smoke_transforms ${_transform_libraries})
  add_test(NAME smoke_transforms COMMAND smoke_transforms)

  # The core's timeouts against rclcpp::Duration, which upstream uses.
  add_executable(smoke_timeouts test/smoke_timeouts.cpp)
  target_link_libraries(smoke_timeouts rclcpp::rclcpp roslean_core::roslean_core)
  add_test(NAME smoke_timeouts COMMAND smoke_timeouts)

  # The core's zones against upstream's zone tests, point by point: the two
  # bounds of SPEC D4 (nothing upstream counts is missed; nothing is counted
  # beyond 3.5 mm / 4.4 mm outside), on 6.4 million points near boundaries.
  add_executable(band_bounds test/band_bounds.cpp)
  target_include_directories(band_bounds PRIVATE include)
  target_link_libraries(band_bounds roslean_core::roslean_core ${_transform_libraries})
  add_test(NAME band_bounds COMMAND band_bounds 20000)

  # What a command costs in the core, against the budget of COMPARISON.md,
  # "Cost of a command" (defect 2, fixed by T03): 50, 150 and 1500 us at
  # 360, 1080 and 10,000 stored readings, two zones of either kind. A
  # threshold on run time: the budget is twice what was measured, and
  # RUN_SERIAL keeps other tests of this package off the processor.
  add_executable(cmd_cost test/cmd_cost.cpp)
  target_include_directories(cmd_cost PRIVATE include)
  target_link_libraries(cmd_cost roslean_core::roslean_core ${_transform_libraries})
  add_test(NAME roslean_cmd_cost COMMAND cmd_cost)
  set_tests_properties(roslean_cmd_cost PROPERTIES RUN_SERIAL TRUE)

  if(APPLE)
    # T2 on macOS: for one angle at which Apple's cosf and __sincosf_stret
    # differ, the shim's point is the one upstream's binary computes. It
    # failed while the node was built without optimisation (review CM3/CM4,
    # finding 5; fixed since) and fails again in such a build.
    add_executable(t2_point_as_upstream test/t2_point_as_upstream.cpp)
    target_include_directories(t2_point_as_upstream PRIVATE include)
    target_link_libraries(t2_point_as_upstream ${_transform_libraries})
    add_test(NAME t2_point_as_upstream COMMAND t2_point_as_upstream)
  endif()

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
  # overlay/nav2_collision_monitor of this repository is this same node under
  # upstream's package name. Comparing the node with itself would prove
  # nothing. (test/cm_harness.py refuses the overlay again when a test runs.)
  if(nav2_collision_monitor_FOUND AND nav2_collision_monitor_IS_ROSLEAN_OVERLAY)
    message(WARNING "the package nav2_collision_monitor that was found is the ROSlean "
      "overlay, not upstream: build and test without the overlay in the environment "
      "to get the comparison with upstream")
    set(nav2_collision_monitor_FOUND FALSE)
  endif()
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

  # ---- The ROSlean node through the same harness -----------------------
  # Results and what they mean: docs/collision_monitor/COMPARISON.md.
  set(_lean "$<TARGET_FILE:collision_monitor>")

  # Every fixed scenario, and the probes of test/probes.py. The expectations
  # are upstream's; test/profile_roslean.py states each answer that differs,
  # with the number of the difference in the SPEC, and the configurations
  # the node must refuse. A stated difference that does not occur fails too.
  add_test(NAME roslean_scenario
    COMMAND "${Python3_EXECUTABLE}" "${_test}/scenario.py" "${_lean}" --profile profile_roslean)
  # LABELS long (five minutes): not part of `pixi run test-quick`.
  set_tests_properties(roslean_scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=91" TIMEOUT 3600
    LABELS long)

  # Malformed and hostile input. Without upstream the node is driven alone.
  if(nav2_collision_monitor_FOUND)
    set(_beside ${_upstream})
  else()
    set(_beside "")
  endif()
  add_test(NAME roslean_fault_injection
    COMMAND "${Python3_EXECUTABLE}" "${_test}/fault_injection.py" "${_lean}" ${_beside}
      --skip clock-negative)
  set_tests_properties(roslean_fault_injection PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=120" TIMEOUT 1800 LABELS long)
  # A /clock message with a negative time (COMPARISON.md, defect 1, fixed by
  # T03; SPEC D15): the node goes on, answers every command with zero while
  # the clock is not valid, and evaluates commands again when it is.
  # Upstream's process ends; when upstream is installed it is run beside the
  # node and must be gone.
  add_test(NAME roslean_fault_clock_negative
    COMMAND "${Python3_EXECUTABLE}" "${_test}/fault_injection.py" "${_lean}" ${_beside}
      --only clock-negative)
  set_tests_properties(roslean_fault_clock_negative PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=121" TIMEOUT 600)

  if(nav2_collision_monitor_FOUND)
    # Statements that only the SPEC makes, put to upstream's binary.
    add_test(NAME upstream_probes
      COMMAND "${Python3_EXECUTABLE}" "${_test}/scenario.py" ${_upstream}
        --profile profile_upstream --own)
    set_tests_properties(upstream_probes PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=92" TIMEOUT 1200)
    # ROSlean against upstream on random cases with points at zone
    # boundaries, judged by the bounds of SPEC D4 (test/cm_bounds.py). The
    # long run is `pixi run cm-diff` (600 seeds).
    add_test(NAME roslean_random_diff
      COMMAND "${Python3_EXECUTABLE}" "${_test}/random_diff.py" "${_lean}" ${_upstream} 0 6
        --roslean)
    set_tests_properties(roslean_random_diff PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=93" TIMEOUT 1800
      LABELS long)
  endif()

  # ---- The Nav2 extension (tasks/T07): lifecycle, state topic, polygon
  # topics, transforms, base shift correction, zones fed by a topic --------
  # What the tests pin down: docs/collision_monitor/UPSTREAM_BEHAVIOUR.md,
  # "The Nav2 extension"; the plan they follow: SPEC_NAV2.md, section 10.
  if(nav2_collision_monitor_FOUND)
    # Fixed scenarios with the outputs observed from upstream.
    add_test(NAME upstream_nav2_scenario
      COMMAND "${Python3_EXECUTABLE}" "${_test}/scenario_nav2.py" ${_upstream})
    set_tests_properties(upstream_nav2_scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=140"
      TIMEOUT 1800 LABELS long)
    # The bond and Nav2's lifecycle manager, on the wall clock (half a minute).
    add_test(NAME upstream_nav2_bond
      COMMAND "${Python3_EXECUTABLE}" "${_test}/bond_manager.py" ${_upstream})
    set_tests_properties(upstream_nav2_bond PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=141" TIMEOUT 300)
    # Random configurations and event sequences against the Python model of
    # random_nav2.py.
    add_test(NAME upstream_nav2_random
      COMMAND "${Python3_EXECUTABLE}" "${_test}/random_nav2.py" ${_upstream} 0 10)
    set_tests_properties(upstream_nav2_random PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=142"
      TIMEOUT 1800 LABELS long)
  endif()

  # The ROSlean node through the same three tests (since tasks/T06 it has the
  # extension). test/profile_roslean_nav2.py states the differences from
  # upstream that a scenario can see (ND4, ND5, ND6, ND11) and the two
  # scenarios that are not run for this node.
  add_test(NAME roslean_nav2_scenario
    COMMAND "${Python3_EXECUTABLE}" "${_test}/scenario_nav2.py" "${_lean}"
      --profile profile_roslean_nav2)
  set_tests_properties(roslean_nav2_scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=143"
    TIMEOUT 1800 LABELS long)
  add_test(NAME roslean_nav2_bond
    COMMAND "${Python3_EXECUTABLE}" "${_test}/bond_manager.py" "${_lean}")
  set_tests_properties(roslean_nav2_bond PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=144" TIMEOUT 300)
  if(nav2_collision_monitor_FOUND)
    # Beside upstream on the same messages; both must give the model's answer.
    add_test(NAME roslean_nav2_random
      COMMAND "${Python3_EXECUTABLE}" "${_test}/random_nav2.py" "${_lean}" ${_upstream} 0 3)
    set_tests_properties(roslean_nav2_random PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=145"
      TIMEOUT 1800 LABELS long)
    # The node as an rclcpp component in a container of the kind Nav2 uses,
    # beside upstream's monitor and Nav2's lifecycle manager in one process
    # (SPEC_NAV2.md, 8.5, ND10). Half a minute, on the wall clock.
    add_test(NAME smoke_component
      COMMAND "${Python3_EXECUTABLE}" "${_test}/smoke_component.py")
    set_tests_properties(smoke_component PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=147" TIMEOUT 300)
    # The same in the two other container kinds of rclcpp_components (review
    # CM5, finding 6): one thread for every node of the container, and a pool
    # of threads. In both the node's callbacks run in threads the Lean runtime
    # did not create (finding 5).
    add_test(NAME smoke_component_single
      COMMAND "${Python3_EXECUTABLE}" "${_test}/smoke_component.py" component_container)
    set_tests_properties(smoke_component_single PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=148"
      TIMEOUT 300)
    add_test(NAME smoke_component_mt
      COMMAND "${Python3_EXECUTABLE}" "${_test}/smoke_component.py" component_container_mt)
    set_tests_properties(smoke_component_mt PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=149"
      TIMEOUT 300)
  endif()
endif()

# ---- begin: fixes after the review CM3/CM4 (docs/REVIEWS.md) ---------------
# Finding 5. Without a build type CMake passes no -O flag, and the node then
# calls cos and sin separately where an optimised build (such as upstream's
# binary) makes one call for both; on macOS arm64 the two round differently
# for about 1 % of the coordinates. Build with optimisation unless told
# otherwise, so that the points of a scan are upstream's bit for bit.
# CMAKE_BUILD_TYPE is read when the build files are generated, so setting it
# here, after the targets were declared, takes effect for them.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "roslean_collision_monitor: no build type given, using Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(BUILD_TESTING)
  # smoke_conversions compares the shim's points, compiled with the flags of
  # the node, with upstream's loop in a translation unit of its own that is
  # compiled with -O2 whatever the build type (see the file).
  target_sources(smoke_conversions PRIVATE test/upstream_points.cpp)
  set_source_files_properties(test/upstream_points.cpp PROPERTIES COMPILE_OPTIONS "-O2")
endif()
# ---- end: fixes after the review CM3/CM4 ----------------------------------

ament_package()
