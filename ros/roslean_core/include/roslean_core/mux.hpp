// C++ wrapper around the Lean twist_mux core (ROSleanCore/FFI.lean).
//
// Threading: a Mux or Builder must not be used from two threads at once (Lean
// reference counts on these objects are not atomic). Under a multi-threaded
// executor, serialise access with a mutex or a mutually exclusive callback
// group.

#ifndef ROSLEAN_CORE__MUX_HPP_
#define ROSLEAN_CORE__MUX_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "roslean_core/ffi.h"

namespace roslean
{

/// Initialises the Lean runtime and the ROSleanCore library. Idempotent and
/// thread-safe; called automatically by Mux::Builder. Throws on every call if
/// initialisation failed.
inline void initialize()
{
  static const bool ok = [] {
      lean_initialize_runtime_module();
      lean_object * res = initialize_ROSlean_ROSleanCore(1 /* builtin */);
      const bool success = lean_io_result_is_ok(res);
      if (!success) {
        lean_io_result_show_error(res);
      }
      lean_dec(res);
      if (success) {
        lean_io_mark_end_initialization();
      }
      return success;
    }();
  if (!ok) {
    throw std::runtime_error("failed to initialise the Lean core library");
  }
}

/// Thrown when the Lean core rejects a configuration.
class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// The arbitration core of twist_mux. All decisions are made in Lean; this
/// class only owns the Lean object.
class Mux
{
public:
  /// Collects inputs and locks. Indices are assigned in order of addition,
  /// separately for inputs and locks, starting at 0.
  class Builder
  {
public:
    Builder()
    {
      initialize();
      obj_ = roslean_mux_builder_new(lean_box(0));
    }
    ~Builder() {if (obj_) {lean_dec(obj_);}}
    Builder(const Builder &) = delete;
    Builder & operator=(const Builder &) = delete;

    /// @param timeout_ns 0 disables expiry.
    uint32_t add_input(uint64_t priority, uint64_t timeout_ns)
    {
      obj_ = roslean_mux_builder_add_input(obj_, priority, timeout_ns);
      return num_inputs_++;
    }

    /// @param timeout_ns 0 disables expiry.
    uint32_t add_lock(uint64_t priority, uint64_t timeout_ns)
    {
      obj_ = roslean_mux_builder_add_lock(obj_, priority, timeout_ns);
      return num_locks_++;
    }

    /// Validates the configuration and creates the mux in its initial state.
    /// On success the builder is left empty. Throws ConfigError if Lean
    /// rejects the configuration; the builder is then unchanged.
    Mux build()
    {
      lean_inc(obj_);  // check consumes one reference
      const uint8_t code = roslean_mux_builder_check(obj_);
      if (code != 0) {
        throw ConfigError(describe(code));
      }
      lean_object * mux = roslean_mux_new(obj_);
      obj_ = roslean_mux_builder_new(lean_box(0));
      num_inputs_ = num_locks_ = 0;
      return Mux(mux);
    }

private:
    static std::string describe(uint8_t code)
    {
      switch (code) {
        case 1:
          return "two inputs have the same priority";
        default:
          return "configuration rejected by the Lean core (code " + std::to_string(code) + ")";
      }
    }

    lean_object * obj_;
    uint32_t num_inputs_{0};
    uint32_t num_locks_{0};
  };

  ~Mux() {if (obj_) {lean_dec(obj_);}}
  Mux(Mux && other) noexcept
  : obj_(std::exchange(other.obj_, nullptr)) {}
  Mux & operator=(Mux && other) noexcept
  {
    if (this != &other) {
      if (obj_) {lean_dec(obj_);}
      obj_ = std::exchange(other.obj_, nullptr);
    }
    return *this;
  }
  Mux(const Mux &) = delete;
  Mux & operator=(const Mux &) = delete;

  /// A message arrived on input `i` at node-clock time `t_ns`.
  /// @return whether to forward that message.
  bool on_input(uint32_t i, uint64_t t_ns)
  {
    require_valid();
    obj_ = roslean_mux_on_input(obj_, i, t_ns);
    lean_inc(obj_);  // the query consumes one reference
    return roslean_mux_forward(obj_) != 0;
  }

  /// A message with the given value arrived on lock `j` at time `t_ns`.
  void on_lock(uint32_t j, bool value, uint64_t t_ns)
  {
    require_valid();
    obj_ = roslean_mux_on_lock(obj_, j, value ? 1 : 0, t_ns);
  }

private:
  explicit Mux(lean_object * obj)
  : obj_(obj) {}

  void require_valid() const
  {
    if (!obj_) {
      throw std::logic_error("use of a moved-from roslean::Mux");
    }
  }

  lean_object * obj_;
};

}  // namespace roslean

#endif  // ROSLEAN_CORE__MUX_HPP_
