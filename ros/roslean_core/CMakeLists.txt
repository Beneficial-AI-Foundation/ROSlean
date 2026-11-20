cmake_minimum_required(VERSION 3.20)
project(roslean_core LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

# This package builds one self-contained shared library, libroslean_core:
# the C code that Lean generates for ROSleanCore, linked statically with the
# Lean runtime and Lean's standard library. Nothing in the install tree refers
# to the Lean toolchain or to this checkout, so the tree can be copied to a
# machine that has neither. Lean (elan) is needed at build time only.

# Root of the lake project: this package lives in <root>/ros/roslean_core.
if(NOT ROSLEAN_ROOT)
  get_filename_component(ROSLEAN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
endif()
set(ROSLEAN_ROOT "${ROSLEAN_ROOT}" CACHE PATH "Root of the ROSlean lake project")

# The toolchain is whatever `lean-toolchain` pins. Recompute its location on
# every configure, and re-configure when the pin changes, so that the headers
# and runtime never lag behind the toolchain that lake compiles with.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ROSLEAN_ROOT}/lean-toolchain")
find_program(LAKE_EXECUTABLE lake HINTS "$ENV{HOME}/.elan/bin" REQUIRED)
find_program(LEAN_EXECUTABLE lean HINTS "$ENV{HOME}/.elan/bin" REQUIRED)
execute_process(
  COMMAND "${LEAN_EXECUTABLE}" --print-prefix
  WORKING_DIRECTORY "${ROSLEAN_ROOT}"
  OUTPUT_VARIABLE LEAN_SYSROOT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
set(LEAN_INCLUDE_DIR "${LEAN_SYSROOT}/include")
if(NOT EXISTS "${LEAN_INCLUDE_DIR}/lean/lean.h")
  message(FATAL_ERROR "lean/lean.h not found in ${LEAN_INCLUDE_DIR}")
endif()

# Lean modules that make up the core, as generated C files. Lake writes them;
# run it once now so that they exist, and again on every build so that colcon
# picks up changes to the Lean sources.
set(_ir "${ROSLEAN_ROOT}/.lake/build/ir")
set(ROSLEAN_MODULES ROSleanCore ROSleanCore/Mux ROSleanCore/FFI ROSleanCore/Noop)
set(_generated)
foreach(_m ${ROSLEAN_MODULES})
  list(APPEND _generated "${_ir}/${_m}.c")
endforeach()
execute_process(
  COMMAND "${LAKE_EXECUTABLE}" build ROSleanCore
  WORKING_DIRECTORY "${ROSLEAN_ROOT}"
  COMMAND_ERROR_IS_FATAL ANY)
add_custom_target(roslean_core_lake ALL
  COMMAND "${LAKE_EXECUTABLE}" build ROSleanCore
  WORKING_DIRECTORY "${ROSLEAN_ROOT}"
  BYPRODUCTS ${_generated}
  COMMENT "lake build ROSleanCore"
  USES_TERMINAL)

add_library(roslean_core SHARED ${_generated})
add_dependencies(roslean_core roslean_core_lake)
set_target_properties(roslean_core PROPERTIES C_VISIBILITY_PRESET default)
# Flags Lean itself compiles generated code with (`leanc --print-cflags`).
target_compile_options(roslean_core PRIVATE
  -O3 -DNDEBUG -ffp-contract=off -fdata-sections -ffunction-sections -w)
target_include_directories(roslean_core SYSTEM PRIVATE "${LEAN_INCLUDE_DIR}")
target_include_directories(roslean_core INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# lean.h is not written for -Wpedantic; dependants get it as a system header.
target_include_directories(roslean_core SYSTEM INTERFACE
  "$<BUILD_INTERFACE:${LEAN_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}/lean_sdk>")

# Static Lean libraries. Init is Lean's standard library, leanrt the runtime;
# the rest are what those two need. Only referenced objects are linked in.
set(_lean_static)
foreach(_lib Init leanrt cadical gmp uv)
  find_library(LEAN_STATIC_${_lib} NAMES "lib${_lib}.a"
    HINTS "${LEAN_SYSROOT}/lib/lean" "${LEAN_SYSROOT}/lib" NO_DEFAULT_PATH)
  if(LEAN_STATIC_${_lib})
    list(APPEND _lean_static "${LEAN_STATIC_${_lib}}")
  else()
    message(STATUS "roslean_core: no static lib${_lib}.a in the Lean toolchain")
  endif()
endforeach()

# Export the functions of ffi.h and the Lean runtime functions that lean.h's
# inline code calls; keep everything else private to the library.
if(APPLE)
  target_link_libraries(roslean_core PRIVATE ${_lean_static} c++)
  target_link_options(roslean_core PRIVATE
    -Wl,-dead_strip
    "-Wl,-exported_symbol,_roslean_*"
    "-Wl,-exported_symbol,_initialize_ROSlean_*"
    "-Wl,-exported_symbol,_lean_*")
  # ROS message targets pull in *_rosidl_generator_py libraries, which expect
  # Python symbols from the host process and abort a C++ node at load time.
  # Make dependants drop every dylib they do not actually reference.
  target_link_options(roslean_core INTERFACE "-Wl,-dead_strip_dylibs")
else()
  # The Lean runtime is C++ built against libc++, which the toolchain ships as
  # static libraries; ROS uses libstdc++. Linking libc++ statically and hiding
  # its symbols keeps the two apart.
  set(_cxx)
  foreach(_lib c++ c++abi)
    find_library(LEAN_STATIC_${_lib} NAMES "lib${_lib}.a"
      HINTS "${LEAN_SYSROOT}/lib" "${LEAN_SYSROOT}/lib/lean" NO_DEFAULT_PATH REQUIRED)
    list(APPEND _cxx "${LEAN_STATIC_${_lib}}")
  endforeach()
  target_link_libraries(roslean_core PRIVATE
    -Wl,--start-group ${_lean_static} ${_cxx} -Wl,--end-group m dl pthread)
  target_link_options(roslean_core PRIVATE
    -Wl,--gc-sections
    "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/cmake/exports.map")
  set_property(TARGET roslean_core APPEND PROPERTY
    LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/exports.map")
endif()

# Fails the build if include/roslean_core/ffi.h disagrees with the Lean
# exports: each generated file is compiled once more after that header.
foreach(_m ${ROSLEAN_MODULES})
  string(MAKE_C_IDENTIFIER "check_signatures_${_m}" _tgt)
  add_library(${_tgt} OBJECT test/check_signatures.c)
  target_compile_definitions(${_tgt} PRIVATE "GENERATED_C=\"${_m}.c\"")
  target_include_directories(${_tgt} PRIVATE include "${_ir}")
  target_include_directories(${_tgt} SYSTEM PRIVATE "${LEAN_INCLUDE_DIR}")
  target_compile_options(${_tgt} PRIVATE -w)  # generated code
  add_dependencies(${_tgt} roslean_core_lake)
endforeach()

install(TARGETS roslean_core EXPORT export_roslean_core
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# The Lean C headers that ffi.h includes, so that dependants can be built
# against the install tree alone.
install(DIRECTORY "${LEAN_INCLUDE_DIR}/lean"
  DESTINATION include/${PROJECT_NAME}/lean_sdk)
install(FILES "${LEAN_SYSROOT}/LICENSE" DESTINATION share/${PROJECT_NAME}/lean_sdk OPTIONAL)
install(DIRECTORY "${LEAN_SYSROOT}/LICENSES" DESTINATION share/${PROJECT_NAME}/lean_sdk OPTIONAL)

if(BUILD_TESTING)
  # Checks the C++ wrapper against the library, without ROS.
  add_executable(test_mux_ffi test/test_mux_ffi.cpp)
  target_compile_options(test_mux_ffi PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_mux_ffi roslean_core)
  add_test(NAME test_mux_ffi COMMAND test_mux_ffi)
endif()

ament_export_targets(export_roslean_core HAS_LIBRARY_TARGET)
ament_package()
