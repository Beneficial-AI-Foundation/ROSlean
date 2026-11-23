// Start-up of the Lean core library, shared by the C++ wrappers.

#ifndef ROSLEAN_CORE__RUNTIME_HPP_
#define ROSLEAN_CORE__RUNTIME_HPP_

#include <stdexcept>

#include "roslean_core/ffi.h"

namespace roslean
{

/// Initialises the Lean runtime and the ROSleanCore library. Idempotent and
/// thread-safe; called automatically by Mux::Builder. Throws on every call if
/// initialisation failed.
inline void initialize()
{
  static const bool ok = roslean_core_initialize() != 0;
  if (!ok) {
    throw std::runtime_error("failed to initialise the Lean core library");
  }
}

}  // namespace roslean

#endif  // ROSLEAN_CORE__RUNTIME_HPP_
