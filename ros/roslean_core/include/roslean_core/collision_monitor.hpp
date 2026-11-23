// C++ wrapper around the Lean collision monitor core
// (core/ROSleanCore/CollisionMonitorFFI.lean). The meaning of every argument
// is laid down in docs/collision_monitor/SPEC.md, "Shim boundary".
//
// This class owns the Lean object and converts nothing. Parameter values are
// passed as they were read: a `double` or `float` goes over as its bit
// pattern (the bytes are copied), an integer as an integer. Lean converts
// them to millimetres and nanoseconds and refuses what the node does not
// accept. No cast in this file depends on the value of a floating-point
// number.
//
// Threading: a CollisionMonitor or Builder must not be used from two threads
// at once (Lean reference counts on these objects are not atomic).

#ifndef ROSLEAN_CORE__COLLISION_MONITOR_HPP_
#define ROSLEAN_CORE__COLLISION_MONITOR_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <utility>

#include "roslean_core/ffi.h"
#include "roslean_core/runtime.hpp"

namespace roslean
{

/// The bit pattern of a double: bytes copied, no conversion.
inline uint64_t bits_of(double v)
{
  uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

/// The bit pattern of a float.
inline uint32_t bits_of(float v)
{
  uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

/// The decision core of the collision monitor. All decisions and all
/// conversions of configuration values are made in Lean.
class CollisionMonitor
{
public:
  /// What a zone demands when it triggers.
  enum class ZoneAction : uint8_t { Stop = 0, Slowdown = 1 };

  /// What to do in answer to a command.
  enum class OutputKind : uint8_t
  {
    Silent = 0,      ///< publish nothing
    Pass = 1,        ///< publish the command unchanged
    Slowdown = 2,    ///< publish the command scaled by `ratio_bits`
    StopZone = 3,    ///< publish zero: stop zone `zone` triggered
    StopSource = 4   ///< publish zero: the scan source has no usable data
  };

  /// Why a configuration is refused.
  enum class ConfigError : uint8_t
  {
    None = 0,               ///< accepted
    SourceTimeout = 1,      ///< the node's `source_timeout`
    OwnSourceTimeout = 2,   ///< the source's own `source_timeout`
    StopPubTimeout = 3,     ///< `stop_pub_timeout`
    Zone = 4                ///< a zone; see zone_error
  };

  /// Why a zone is refused.
  enum class ZoneError : uint8_t
  {
    None = 0,       ///< the zone is accepted
    Radius = 1,     ///< not finite, or 1000 m or more in magnitude
    Points = 2,     ///< a coordinate that is not finite or beyond 999.997 m,
                    ///< or fewer than four vertices
    Ratio = 3,      ///< a slowdown ratio with the sign bit set
    MinPoints = 4,  ///< outside the range of a 32-bit int
    NoSuchZone = 5
  };

  struct Output
  {
    OutputKind kind;
    uint32_t zone;        ///< zone index for Slowdown and StopZone, else 0
    uint64_t ratio_bits;  ///< binary64 pattern of the ratio for Slowdown, else 0
  };

  /// Collects the configuration. Zones are indexed in order of addition,
  /// starting at 0. A new builder has no zones, an enabled source and both
  /// timeouts 0.
  class Builder
  {
public:
    Builder()
    {
      initialize();
      obj_ = roslean_cm_builder_new(roslean_core_unit());
    }
    ~Builder() {if (obj_) {roslean_core_dec(obj_);}}
    Builder(const Builder &) = delete;
    Builder & operator=(const Builder &) = delete;

    /// @param node_timeout_s the node's `source_timeout` parameter, seconds.
    /// @param own_timeout_s the source's own `source_timeout`, if it is set.
    void set_source(bool enabled, double node_timeout_s, std::optional<double> own_timeout_s)
    {
      obj_ = roslean_cm_builder_set_source(
        obj_, enabled ? 1 : 0, bits_of(node_timeout_s), own_timeout_s.has_value() ? 1 : 0,
        bits_of(own_timeout_s.value_or(0.0)));
    }

    /// @param timeout_s the `stop_pub_timeout` parameter, seconds.
    void set_stop_pub_timeout(double timeout_s)
    {
      obj_ = roslean_cm_builder_set_stop_pub_timeout(obj_, bits_of(timeout_s));
    }

    /// @param radius_m the `radius` parameter, metres.
    /// @param slowdown_ratio the `slowdown_ratio` parameter; ignored for Stop.
    /// @param min_points the `min_points` parameter.
    /// @return the index of the new zone.
    uint32_t add_circle(
      double radius_m, ZoneAction action, double slowdown_ratio, int64_t min_points, bool enabled)
    {
      obj_ = roslean_cm_builder_add_circle(
        obj_, bits_of(radius_m), static_cast<uint8_t>(action), bits_of(slowdown_ratio),
        static_cast<uint64_t>(min_points), enabled ? 1 : 0);
      return num_zones_++;
    }

    /// Adds a polygon zone without vertices; add them with add_vertex.
    /// @return the index of the new zone.
    uint32_t add_polygon(ZoneAction action, double slowdown_ratio, int64_t min_points, bool enabled)
    {
      obj_ = roslean_cm_builder_add_polygon(
        obj_, static_cast<uint8_t>(action), bits_of(slowdown_ratio),
        static_cast<uint64_t>(min_points), enabled ? 1 : 0);
      return num_zones_++;
    }

    /// Appends a vertex, in metres as parsed from `points`, to the zone added
    /// last; no effect unless that zone is a polygon.
    void add_vertex(float x_m, float y_m)
    {
      obj_ = roslean_cm_builder_add_vertex(obj_, bits_of(x_m), bits_of(y_m));
    }

    /// Creates the monitor in its initial state and leaves the builder as
    /// new. Every configuration yields a monitor; one that is not ok() stops
    /// every command.
    CollisionMonitor build()
    {
      lean_object * monitor = roslean_cm_new(obj_);
      obj_ = roslean_cm_builder_new(roslean_core_unit());
      num_zones_ = 0;
      return CollisionMonitor(monitor);
    }

private:
    lean_object * obj_;
    uint32_t num_zones_{0};
  };

  ~CollisionMonitor() {if (obj_) {roslean_core_dec(obj_);}}
  CollisionMonitor(CollisionMonitor && other) noexcept
  : obj_(std::exchange(other.obj_, nullptr)) {}
  CollisionMonitor & operator=(CollisionMonitor && other) noexcept
  {
    if (this != &other) {
      if (obj_) {roslean_core_dec(obj_);}
      obj_ = std::exchange(other.obj_, nullptr);
    }
    return *this;
  }
  CollisionMonitor(const CollisionMonitor &) = delete;
  CollisionMonitor & operator=(const CollisionMonitor &) = delete;

  /// Whether the configuration is accepted. A node must not run otherwise;
  /// if it does, every command is answered with StopSource.
  bool ok() const
  {
    require_valid();
    roslean_core_inc(obj_);  // the query consumes one reference
    return roslean_cm_ok(obj_) != 0;
  }

  /// Why the configuration is refused. For error messages.
  ConfigError error() const
  {
    require_valid();
    roslean_core_inc(obj_);
    return static_cast<ConfigError>(roslean_cm_error(obj_));
  }

  /// Why zone `k` is refused. For error messages.
  ZoneError zone_error(uint32_t k) const
  {
    require_valid();
    roslean_core_inc(obj_);
    return static_cast<ZoneError>(roslean_cm_zone_error(obj_, k));
  }

  /// A scan arrived at node-clock time `now_ns`.
  /// @param stamp_ns the header stamp.
  /// @param usable false if the scan's points cannot be placed in the base
  ///   frame; the stored scan is then discarded and this one is not stored.
  /// @param range_min_bits, range_max_bits binary32 patterns of the limits.
  /// @param beams `size` bytes: one 20-byte record per reading (range
  ///   pattern, 4 bytes; x and y in metres as binary64 patterns, 8 bytes
  ///   each; all little-endian). The bytes are copied. A trailing partial
  ///   record is ignored.
  void on_scan(
    uint64_t now_ns, int64_t stamp_ns, bool usable, uint32_t range_min_bits,
    uint32_t range_max_bits, const uint8_t * beams, size_t size)
  {
    require_valid();
    obj_ = roslean_cm_on_scan(
      obj_, now_ns, static_cast<uint64_t>(stamp_ns), usable ? 1 : 0, range_min_bits,
      range_max_bits, roslean_core_byte_array(beams, size));
  }

  /// A velocity command arrived at node-clock time `now_ns`; the six
  /// components are binary64 bit patterns.
  /// @return what to publish.
  Output on_cmd(
    uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
    uint64_t ang_x, uint64_t ang_y, uint64_t ang_z)
  {
    require_valid();
    obj_ = roslean_cm_on_cmd(obj_, now_ns, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z);
    Output out;
    roslean_core_inc(obj_);  // each query consumes one reference
    out.kind = static_cast<OutputKind>(roslean_cm_out_kind(obj_));
    roslean_core_inc(obj_);
    out.zone = roslean_cm_out_zone(obj_);
    roslean_core_inc(obj_);
    out.ratio_bits = roslean_cm_out_ratio(obj_);
    return out;
  }

  // The conversions Lean applies, on their own: for tests and log messages.

  /// A scan coordinate in metres to millimetres: truncated toward zero,
  /// saturated at ±1,000,000; NaN is +1,000,000.
  static int64_t coord_mm(double metres)
  {
    initialize();
    return static_cast<int64_t>(roslean_cm_coord_mm(bits_of(metres)));
  }

  /// A circle's radius in millimetres (magnitude, rounded up, plus 1), or
  /// nothing if it is refused.
  static std::optional<int64_t> radius_mm(double metres)
  {
    initialize();
    const auto r = static_cast<int64_t>(roslean_cm_radius_mm(bits_of(metres)));
    return r < 0 ? std::nullopt : std::optional<int64_t>(r);
  }

  /// A polygon vertex coordinate in millimetres (rounded to nearest), or
  /// nothing if it is refused. Limited to ±2^62.
  static std::optional<int64_t> vertex_mm(float metres)
  {
    initialize();
    const auto v = static_cast<int64_t>(roslean_cm_vertex_mm(bits_of(metres)));
    return v == INT64_MIN ? std::nullopt : std::optional<int64_t>(v);
  }

  /// A timeout in nanoseconds, or nothing if it is refused.
  static std::optional<int64_t> timeout_ns(double seconds)
  {
    initialize();
    const auto t = static_cast<int64_t>(roslean_cm_timeout_ns(bits_of(seconds)));
    return t < 0 ? std::nullopt : std::optional<int64_t>(t);
  }

  /// The source's timeout in nanoseconds as set_source computes it, or
  /// nothing if a value is refused.
  static std::optional<int64_t> source_timeout_ns(
    double node_timeout_s, std::optional<double> own_timeout_s)
  {
    initialize();
    const auto t = static_cast<int64_t>(roslean_cm_source_timeout_ns(
        bits_of(node_timeout_s), own_timeout_s.has_value() ? 1 : 0,
        bits_of(own_timeout_s.value_or(0.0))));
    return t < 0 ? std::nullopt : std::optional<int64_t>(t);
  }

private:
  explicit CollisionMonitor(lean_object * obj)
  : obj_(obj) {}

  void require_valid() const
  {
    if (!obj_) {
      throw std::logic_error("use of a moved-from roslean::CollisionMonitor");
    }
  }

  lean_object * obj_;
};

}  // namespace roslean

#endif  // ROSLEAN_CORE__COLLISION_MONITOR_HPP_
