// Start-up of the Lean core library, shared by the C++ wrappers.

#ifndef ROSLEAN_CORE__RUNTIME_HPP_
#define ROSLEAN_CORE__RUNTIME_HPP_

#include <stdexcept>
#include <thread>

#include "roslean_core/ffi.h"

namespace roslean
{

namespace detail
{

/// The thread that initialised the Lean runtime (roslean_core_initialize
/// prepares that thread itself).
inline std::thread::id & runtime_thread()
{
  static std::thread::id id;
  return id;
}

inline bool initialize_runtime()
{
  runtime_thread() = std::this_thread::get_id();
  return roslean_core_initialize() != 0;
}

/// One per thread that calls into Lean: `lean_initialize_thread` when the
/// thread first does so, `lean_finalize_thread` when the thread ends.
struct LeanThread
{
  LeanThread()
  : foreign_(std::this_thread::get_id() != runtime_thread())
  {
    if (foreign_) {
      roslean_core_initialize_thread();
    }
  }
  ~LeanThread()
  {
    if (foreign_) {
      roslean_core_finalize_thread();
    }
  }
  bool foreign_;
};

}  // namespace detail

/// Initialises the Lean runtime and the ROSleanCore library, once, and
/// prepares the calling thread for Lean code, once per thread. Idempotent and
/// thread-safe; every function of the C++ wrappers that calls into Lean
/// calls it first, so that a wrapper can be used from any thread (a node
/// loaded as a component runs in the threads of its container's executor).
/// Throws on every call if initialisation failed.
///
/// The Lean runtime requires `lean_initialize_thread()` in every thread that
/// runs Lean code and was not created by Lean (review CM5, finding 5). In
/// the toolchain this repository pins (4.37.0-rc1, built with mimalloc) that
/// function only reads an environment variable and `lean_finalize_thread`
/// is empty (read from the disassembly of libleanrt.a); a toolchain built
/// with Lean's own small-object allocator sets up the thread's heap there.
///
/// Not covered by this: Lean objects are reference-counted without atomic
/// operations unless marked as shared between threads, which the wrappers do
/// not do. An object may move from thread to thread, but two threads must
/// never be inside the wrappers with the same object at once. The nodes
/// guarantee that with one mutually exclusive callback group.
inline void initialize()
{
  static const bool ok = detail::initialize_runtime();
  if (!ok) {
    throw std::runtime_error("failed to initialise the Lean core library");
  }
  thread_local const detail::LeanThread lean_thread;
  (void)lean_thread;
}

}  // namespace roslean

#endif  // ROSLEAN_CORE__RUNTIME_HPP_
