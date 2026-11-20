# Locates the Lean toolchain and the ROSleanCore shared library built by
# `lake build ROSleanCore:shared`, and defines the imported target
# `ROSlean::Core`. Reusable by every ROS package in this repo.
#
# Inputs (cache variables, all optional):
#   ROSLEAN_ROOT  - root of the lake project (default: two levels above the package)
#   LEAN_SYSROOT  - Lean toolchain prefix (default: `lean --print-prefix` run in ROSLEAN_ROOT)

if(NOT ROSLEAN_ROOT)
  get_filename_component(ROSLEAN_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." REALPATH)
endif()
set(ROSLEAN_ROOT "${ROSLEAN_ROOT}" CACHE PATH "Root of the ROSlean lake project")

if(NOT LEAN_SYSROOT)
  # Run inside the lake project so elan picks the version pinned in lean-toolchain.
  find_program(LEAN_EXECUTABLE lean HINTS "$ENV{HOME}/.elan/bin" REQUIRED)
  execute_process(
    COMMAND "${LEAN_EXECUTABLE}" --print-prefix
    WORKING_DIRECTORY "${ROSLEAN_ROOT}"
    OUTPUT_VARIABLE LEAN_SYSROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY)
endif()
set(LEAN_SYSROOT "${LEAN_SYSROOT}" CACHE PATH "Lean toolchain prefix")

set(_lean_libdir "${LEAN_SYSROOT}/lib/lean")
set(_core_libdir "${ROSLEAN_ROOT}/.lake/build/lib")

find_path(LEAN_INCLUDE_DIR lean/lean.h HINTS "${LEAN_SYSROOT}/include" NO_DEFAULT_PATH REQUIRED)
# Lake names the library <package>_<lib>.
find_library(ROSLEAN_CORE_LIBRARY ROSlean_ROSleanCore HINTS "${_core_libdir}" NO_DEFAULT_PATH)
if(NOT ROSLEAN_CORE_LIBRARY)
  message(FATAL_ERROR
    "libROSlean_ROSleanCore not found in ${_core_libdir}. "
    "Run `lake build ROSleanCore:shared` in ${ROSLEAN_ROOT} first (or `pixi run build`).")
endif()
# The core library leaves the Lean runtime symbols undefined (dynamic lookup),
# so the executable itself must load the runtime and Init.
find_library(LEAN_SHARED_LIBRARY leanshared HINTS "${_lean_libdir}" NO_DEFAULT_PATH REQUIRED)
find_library(LEAN_INIT_SHARED_LIBRARY Init_shared HINTS "${_lean_libdir}" NO_DEFAULT_PATH REQUIRED)

if(NOT TARGET ROSlean::Core)
  add_library(ROSlean::Core INTERFACE IMPORTED)
  # SYSTEM: lean.h is not warning-clean under -Wpedantic.
  set_target_properties(ROSlean::Core PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${LEAN_INCLUDE_DIR}"
    INTERFACE_INCLUDE_DIRECTORIES "${LEAN_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES
      "${ROSLEAN_CORE_LIBRARY};${LEAN_INIT_SHARED_LIBRARY};${LEAN_SHARED_LIBRARY}")
endif()

set(ROSLEAN_RPATH "${_core_libdir};${_lean_libdir}")
