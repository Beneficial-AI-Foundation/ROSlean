cmake_minimum_required(VERSION 3.16)
project(roslean_demos NONE)

find_package(ament_cmake REQUIRED)

# turtlesim/        the mux in turtlesim (docs/TUTORIAL.md)
# gazebo/mux/       the mux on a TurtleBot3 in Gazebo: world, map, patrol,
#                   base watchdog, Nav2 parameters (docs/TUTORIAL_GAZEBO.md)
# gazebo/monitor/   the collision monitor behind the mux and in Nav2's
#                   bring-up: box scenario, check_demo.py, nav2_observe.py
#                   (docs/TUTORIAL_COLLISION_MONITOR.md)
# Nothing is compiled. check_demo.py is not a colcon test: it needs the
# simulator with a display and the downloaded furniture (`pixi run cm-check`).
install(DIRECTORY turtlesim gazebo DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS PATTERN "__pycache__" EXCLUDE)

ament_package()
