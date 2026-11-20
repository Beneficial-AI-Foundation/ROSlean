# Defines the imported target `roslean_core::lean`: the Lean runtime plus the
# ROSleanCore library built by lake. Included both by this package's own
# CMakeLists.txt and, after installation, by every package that calls
# `find_package(roslean_core)`.
#
# Requires ROSLEAN_ROOT (root of the lake project). Optional: LEAN_SYSROOT.

if(TARGET roslean_core::lean)
  return()
endif()

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
    "Run `lake build ROSleanCore:shared` in ${ROSLEAN_ROOT}.")
endif()
# The core library leaves the Lean runtime symbols undefined (dynamic lookup),
# so the executable itself must load the runtime.
find_library(LEAN_SHARED_LIBRARY leanshared HINTS "${_lean_libdir}" NO_DEFAULT_PATH REQUIRED)

add_library(roslean_core::lean INTERFACE IMPORTED)
# SYSTEM: lean.h is not written for -Wpedantic.
set_target_properties(roslean_core::lean PROPERTIES
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${LEAN_INCLUDE_DIR}"
  INTERFACE_INCLUDE_DIRECTORIES "${LEAN_INCLUDE_DIR}"
  INTERFACE_LINK_LIBRARIES "${ROSLEAN_CORE_LIBRARY};${LEAN_SHARED_LIBRARY}")
if(APPLE)
  # ROS message targets pull in *_rosidl_generator_py libraries, which expect
  # Python symbols from the host process and abort a C++ node at load time.
  # Drop every dylib the executable does not actually reference.
  set_property(TARGET roslean_core::lean PROPERTY
    INTERFACE_LINK_OPTIONS "-Wl,-dead_strip_dylibs")
endif()

# The Lean libraries live outside any install tree. Executables linking
# roslean_core must carry these directories in their rpath:
#   set_target_properties(<exe> PROPERTIES INSTALL_RPATH "${ROSLEAN_RPATH}")
set(ROSLEAN_RPATH "${_core_libdir};${_lean_libdir}")
