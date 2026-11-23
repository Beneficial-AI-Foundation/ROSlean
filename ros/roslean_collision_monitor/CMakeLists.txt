cmake_minimum_required(VERSION 3.20)
project(roslean_collision_monitor LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)

# The tests and their configuration, so that they can be run from the install
# tree against any node:
#   python3 <prefix>/share/roslean_collision_monitor/test/scenario.py <package>/<executable>
install(DIRECTORY test DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  # Black-box tests under simulated time; what they pin down is described in
  # docs/collision_monitor/UPSTREAM_BEHAVIOUR.md. The node under test is the
  # first argument of each script. To run a node of this package through the
  # same tests, add a test with "$<TARGET_FILE:the_node>" in place of
  # ${_upstream} (and --no-tf if it uses no transforms, as the reference
  # node below). Lifecycle nodes are detected and brought to the active state.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
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
  set_tests_properties(reference_scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=91" TIMEOUT 1800)

  if(nav2_collision_monitor_FOUND)
    # Fixed scenarios with the outputs observed from upstream.
    add_test(NAME upstream_scenario
      COMMAND "${Python3_EXECUTABLE}" "${_test}/scenario.py" ${_upstream})
    set_tests_properties(upstream_scenario PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=89" TIMEOUT 3600)
    # Random configurations and event sequences: upstream against itself
    # (which measures nondeterminism) and against the Python model.
    add_test(NAME upstream_random_diff
      COMMAND "${Python3_EXECUTABLE}" "${_test}/random_diff.py" ${_upstream} ${_upstream} 0 20)
    set_tests_properties(upstream_random_diff PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=90" TIMEOUT 2400)
    # The same with two different implementations side by side.
    add_test(NAME reference_random_diff
      COMMAND "${Python3_EXECUTABLE}" "${_test}/random_diff.py" "${_reference}" ${_upstream} 0 5
        --no-tf)
    set_tests_properties(reference_random_diff PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=92" TIMEOUT 1200)
  else()
    message(WARNING "nav2_collision_monitor not found: the tests against upstream are not registered")
  endif()
endif()

ament_package()
