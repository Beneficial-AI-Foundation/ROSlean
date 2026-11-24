cmake_minimum_required(VERSION 3.20)
project(roslean_core LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

# This package builds the shared library libroslean_core from the C code that
# Lean generates for ROSleanCore, and installs everything it needs at run
# time next to it. Nothing in the install tree refers to the Lean toolchain or
# to this checkout, so the tree can be copied to a machine that has neither.
# Lean (elan) is needed at build time only.
#
# How the Lean runtime gets in differs by platform:
#   macOS  linked statically into libroslean_core (about 4 MB).
#   Linux  Lean's static runtime cannot go into a shared library there (it is
#          compiled for thread-local storage in executables only), so
#          libroslean_core links Lean's own shared runtime, libleanshared.so,
#          and a stripped copy of that is installed alongside (about 190 MB).

# The lake package of the core: <root>/core, next to <root>/ros.
if(NOT ROSLEAN_ROOT)
  get_filename_component(ROSLEAN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../core" REALPATH)
endif()
set(ROSLEAN_ROOT "${ROSLEAN_ROOT}" CACHE PATH "The ROSleanCore lake package")

# The proofs (parent package) must be checked with the toolchain that compiles
# the core.
if(EXISTS "${ROSLEAN_ROOT}/../lean-toolchain")
  file(READ "${ROSLEAN_ROOT}/lean-toolchain" _tc_core)
  file(READ "${ROSLEAN_ROOT}/../lean-toolchain" _tc_proofs)
  if(NOT _tc_core STREQUAL _tc_proofs)
    message(FATAL_ERROR "core/lean-toolchain and lean-toolchain differ")
  endif()
endif()

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
set(ROSLEAN_MODULES
  ROSleanCore ROSleanCore/Mux ROSleanCore/FFI ROSleanCore/Noop
  ROSleanCore/CollisionMonitorXform
  ROSleanCore/CollisionMonitor ROSleanCore/CollisionMonitorConvert
  ROSleanCore/CollisionMonitorFFI)
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

add_library(roslean_core SHARED ${_generated} src/runtime.c)
add_dependencies(roslean_core roslean_core_lake)
set_target_properties(roslean_core PROPERTIES C_VISIBILITY_PRESET default)
# Flags Lean itself compiles generated code with (`leanc --print-cflags`).
target_compile_options(roslean_core PRIVATE
  -O3 -DNDEBUG -ffp-contract=off -fdata-sections -ffunction-sections)
set_source_files_properties(${_generated} PROPERTIES COMPILE_OPTIONS -w)
target_include_directories(roslean_core PRIVATE include)
target_include_directories(roslean_core SYSTEM PRIVATE "${LEAN_INCLUDE_DIR}")
target_include_directories(roslean_core INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

if(APPLE)
  # Static Lean libraries. Init is Lean's standard library, leanrt the
  # runtime; the rest are what those two need. Only referenced objects are
  # linked in.
  set(_lean_static)
  foreach(_lib Init leanrt cadical gmp uv)
    find_library(LEAN_STATIC_${_lib} NAMES "lib${_lib}.a"
      HINTS "${LEAN_SYSROOT}/lib/lean" "${LEAN_SYSROOT}/lib" NO_DEFAULT_PATH REQUIRED)
    list(APPEND _lean_static "${LEAN_STATIC_${_lib}}")
  endforeach()
  target_link_libraries(roslean_core PRIVATE ${_lean_static} c++)
  # Export the functions of ffi.h; keep everything else private.
  target_link_options(roslean_core PRIVATE
    -Wl,-dead_strip
    "-Wl,-exported_symbol,_roslean_*"
    "-Wl,-exported_symbol,_initialize_ROSleanCore_*")
  # ROS message targets pull in *_rosidl_generator_py libraries, which expect
  # Python symbols from the host process and abort a C++ node at load time.
  # Make dependants drop every dylib they do not actually reference.
  target_link_options(roslean_core INTERFACE "-Wl,-dead_strip_dylibs")
else()
  # libleanshared.so holds the runtime, Lean's standard library and its own
  # private copy of libc++; it needs only the C library. Link a stripped copy
  # that sits next to libroslean_core, found through $ORIGIN.
  #
  # libleanshared.so also exports an exception unwinder of its own
  # (_Unwind_*). If the dynamic loader finds that one before the system's
  # libgcc_s, C++ exceptions in the whole process stop being caught. Two
  # measures keep it behind libgcc_s in the loader's search order:
  #   - dependants never link libleanshared directly (PRIVATE below; they
  #     call the Lean runtime only through src/runtime.c), so it is loaded
  #     one level below everything the executable links;
  #   - libroslean_core itself lists libgcc_s before libleanshared.
  find_library(LEAN_SHARED_LIBRARY NAMES leanshared
    HINTS "${LEAN_SYSROOT}/lib/lean" NO_DEFAULT_PATH REQUIRED)
  set(_leanshared "${CMAKE_CURRENT_BINARY_DIR}/libleanshared.so")
  add_custom_command(OUTPUT "${_leanshared}"
    COMMAND "${CMAKE_STRIP}" --strip-unneeded -o "${_leanshared}" "${LEAN_SHARED_LIBRARY}"
    DEPENDS "${LEAN_SHARED_LIBRARY}"
    COMMENT "Copying and stripping libleanshared.so")
  add_custom_target(roslean_core_leanshared DEPENDS "${_leanshared}")
  add_dependencies(roslean_core roslean_core_leanshared)
  target_link_libraries(roslean_core PRIVATE
    -Wl,--no-as-needed gcc_s -Wl,--as-needed "${_leanshared}")
  target_link_options(roslean_core PRIVATE
    -Wl,--no-undefined
    "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/cmake/exports.map")
  set_target_properties(roslean_core PROPERTIES
    LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/exports.map"
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}"
    INSTALL_RPATH "$ORIGIN")
  install(FILES "${_leanshared}" DESTINATION lib)
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
# Licences of Lean and of the libraries linked into libroslean_core.
foreach(_lic LICENSE LICENSES)
  if(IS_DIRECTORY "${LEAN_SYSROOT}/${_lic}")
    install(DIRECTORY "${LEAN_SYSROOT}/${_lic}" DESTINATION share/${PROJECT_NAME}/lean_sdk)
  elseif(EXISTS "${LEAN_SYSROOT}/${_lic}")
    install(FILES "${LEAN_SYSROOT}/${_lic}" DESTINATION share/${PROJECT_NAME}/lean_sdk)
  endif()
endforeach()

if(BUILD_TESTING)
  # Checks the C++ wrapper against the library, without ROS.
  add_executable(test_mux_ffi test/test_mux_ffi.cpp)
  target_compile_options(test_mux_ffi PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(test_mux_ffi roslean_core)
  add_test(NAME test_mux_ffi COMMAND test_mux_ffi)
  add_executable(test_cm_ffi test/test_cm_ffi.cpp)
  # -ffp-contract=off: the reference computations in the test must be the
  # same IEEE operations on every platform.
  target_compile_options(test_cm_ffi PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
  target_link_libraries(test_cm_ffi roslean_core)
  add_test(NAME test_cm_ffi COMMAND test_cm_ffi)
endif()

ament_export_targets(export_roslean_core HAS_LIBRARY_TARGET)
ament_package()
