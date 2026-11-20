cmake_minimum_required(VERSION 3.20)
project(roslean_core LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Root of the lake project: this package lives in <root>/ros/roslean_core.
if(NOT ROSLEAN_ROOT)
  get_filename_component(ROSLEAN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
endif()
set(ROSLEAN_ROOT "${ROSLEAN_ROOT}" CACHE PATH "Root of the ROSlean lake project")

# Build the Lean library: once now, so that it can be located, and again on
# every build, so that colcon picks up changes to the Lean sources. The
# library is loaded dynamically, so dependants need no relink.
find_program(LAKE_EXECUTABLE lake HINTS "$ENV{HOME}/.elan/bin" REQUIRED)
execute_process(
  COMMAND "${LAKE_EXECUTABLE}" build ROSleanCore:shared
  WORKING_DIRECTORY "${ROSLEAN_ROOT}"
  COMMAND_ERROR_IS_FATAL ANY)
add_custom_target(roslean_core_lake ALL
  COMMAND "${LAKE_EXECUTABLE}" build ROSleanCore:shared
  WORKING_DIRECTORY "${ROSLEAN_ROOT}"
  COMMENT "lake build ROSleanCore:shared"
  USES_TERMINAL)

include(cmake/lean_core.cmake)

# Fails the build if include/roslean_core/ffi.h disagrees with the Lean
# exports. List every generated file that defines an exported function.
foreach(_gen ROSleanCore ROSleanCore/Noop ROSleanCore/FFI)
  string(MAKE_C_IDENTIFIER "check_signatures_${_gen}" _tgt)
  add_library(${_tgt} OBJECT test/check_signatures.c)
  target_compile_definitions(${_tgt} PRIVATE "GENERATED_C=\"${_gen}.c\"")
  target_include_directories(${_tgt} PRIVATE include "${ROSLEAN_ROOT}/.lake/build/ir")
  target_include_directories(${_tgt} SYSTEM PRIVATE "${LEAN_INCLUDE_DIR}")
  target_compile_options(${_tgt} PRIVATE -w)  # generated code
  add_dependencies(${_tgt} roslean_core_lake)
endforeach()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES cmake/lean_core.cmake DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)
  # Checks the C++ wrapper against the Lean exports, without ROS.
  add_executable(test_mux_ffi test/test_mux_ffi.cpp)
  target_include_directories(test_mux_ffi PRIVATE include)
  target_link_libraries(test_mux_ffi roslean_core::lean)
  add_dependencies(test_mux_ffi roslean_core_lake)
  set_target_properties(test_mux_ffi PROPERTIES BUILD_RPATH "${ROSLEAN_RPATH}")
  add_test(NAME test_mux_ffi COMMAND test_mux_ffi)
endif()

ament_package(CONFIG_EXTRAS cmake/roslean_core-extras.cmake.in)
