cmake_minimum_required(VERSION 3.16)
project(roslean_nav2_bringup NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# roslean_nav2_bringup/derive.py: what the two launch files share.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
# ros2 run roslean_nav2_bringup sync_params.py [--update]
install(PROGRAMS scripts/sync_params.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_program(_python3 python3 REQUIRED)
  # Fails when params/*.yaml differ from nav2_bringup's installed
  # nav2_params.yaml anywhere outside the collision_monitor section: a Nav2
  # update changed the defaults, and `sync_params.py --update` takes them over.
  add_test(NAME params_follow_nav2
    COMMAND ${_python3} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/sync_params.py
            ${CMAKE_CURRENT_SOURCE_DIR}/params)
  # The edit of nav2_bringup's launch files: found on the installed files,
  # refused on files that are not as expected; the parameter files of this
  # package pass the check the launch files run, Nav2's own does not.
  add_test(NAME derive
    COMMAND ${_python3} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_derive.py)
  set_tests_properties(derive PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
endif()

ament_package()
