// A collision monitor core that is handed each scan together with the
// base-frame coordinates of its readings, for tests and benchmarks.
//
// The C interface takes the ranges with the scan and the coordinates with
// every command (SPEC_NAV2.md, 8.2 and 8.3). A test that only wants to say
// "these readings lie at these points" would have to repeat the coordinates
// with every command. This class plays that part of the shim: it keeps the
// coordinates of the scan handed over last and passes them with each command
// (or "none" after a scan given as not usable, which stands for a failed
// transform lookup). A monitor built here is configured and activated at
// once. It adds nothing else; every decision is the core's.
//
// The node does not use this file.

#ifndef ROSLEAN_CORE__SCAN_POINTS_MONITOR_HPP_
#define ROSLEAN_CORE__SCAN_POINTS_MONITOR_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "roslean_core/collision_monitor.hpp"

namespace roslean
{

class ScanPointsMonitor
{
public:
  using ZoneAction = CollisionMonitor::ZoneAction;
  using OutputKind = CollisionMonitor::OutputKind;
  using ConfigError = CollisionMonitor::ConfigError;
  using ZoneError = CollisionMonitor::ZoneError;
  using Output = CollisionMonitor::Output;
  using Report = CollisionMonitor::Report;
  using Transition = CollisionMonitor::Transition;
  using Phase = CollisionMonitor::Phase;
  using Plan = CollisionMonitor::Plan;

  /// The size of a reading's record as on_scan takes it: the range pattern
  /// (4 bytes), then x and y in metres in the base frame as binary64
  /// patterns (8 bytes each), all little-endian.
  static constexpr size_t kRecord = 20;

  class Builder : public CollisionMonitor::Builder
  {
public:
    /// The monitor, configured and activated.
    ScanPointsMonitor build()
    {
      CollisionMonitor m = CollisionMonitor::Builder::build();
      m.on_lifecycle(Transition::Configure);
      m.on_lifecycle(Transition::Activate);
      return ScanPointsMonitor(std::move(m));
    }
  };

  explicit ScanPointsMonitor(CollisionMonitor && m)
  : m_(std::move(m)) {}

  /// Takes ownership of a monitor object of the C interface (as built: the
  /// lifecycle transitions are the caller's).
  explicit ScanPointsMonitor(lean_object * obj)
  : m_(obj) {}

  CollisionMonitor & core() {return m_;}

  bool ok() const {return m_.ok();}
  ConfigError error() const {return m_.error();}
  ZoneError zone_error(uint32_t k) const {return m_.zone_error(k);}
  Phase phase() const {return m_.phase();}
  void on_lifecycle(Transition t) {m_.on_lifecycle(t);}

  /// A scan with the coordinates of its readings: `size` bytes of 20-byte
  /// records; a trailing partial record is ignored. `usable` false: the
  /// scan is stored, and the following commands carry no coordinates.
  void on_scan(
    uint64_t now_ns, int64_t stamp_ns, bool usable, uint32_t range_min_bits,
    uint32_t range_max_bits, const uint8_t * beams, size_t size)
  {
    const size_t n = size / kRecord;
    ranges_.resize(4 * n);
    points_.resize(16 * n);
    for (size_t i = 0; i < n; ++i) {
      std::memcpy(ranges_.data() + 4 * i, beams + kRecord * i, 4);
      std::memcpy(points_.data() + 16 * i, beams + kRecord * i + 4, 16);
    }
    has_points_ = usable;
    m_.on_scan(now_ns, stamp_ns, range_min_bits, range_max_bits, ranges_.data(), ranges_.size());
  }

  Output on_cmd(
    uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
    uint64_t ang_x, uint64_t ang_y, uint64_t ang_z)
  {
    return m_.on_cmd(
      now_ns, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z, has_points_, points_.data(),
      points_.size(), nullptr, 0);
  }

  Output on_cmd_reference(
    uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
    uint64_t ang_x, uint64_t ang_y, uint64_t ang_z)
  {
    return m_.on_cmd_reference(
      now_ns, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z, has_points_, points_.data(),
      points_.size(), nullptr, 0);
  }

  Output on_cmd_no_clock(
    uint64_t lin_x, uint64_t lin_y, uint64_t lin_z, uint64_t ang_x, uint64_t ang_y,
    uint64_t ang_z)
  {
    return m_.on_cmd_no_clock(lin_x, lin_y, lin_z, ang_x, ang_y, ang_z);
  }

  static int64_t coord_mm(double metres) {return CollisionMonitor::coord_mm(metres);}
  static std::optional<int64_t> radius_mm(double metres)
  {
    return CollisionMonitor::radius_mm(metres);
  }
  static std::optional<int64_t> vertex_mm(float metres)
  {
    return CollisionMonitor::vertex_mm(metres);
  }
  static std::optional<int64_t> timeout_ns(double seconds)
  {
    return CollisionMonitor::timeout_ns(seconds);
  }
  static std::optional<int64_t> source_timeout_ns(
    double node_timeout_s, std::optional<double> own_timeout_s)
  {
    return CollisionMonitor::source_timeout_ns(node_timeout_s, own_timeout_s);
  }

private:
  CollisionMonitor m_;
  std::vector<uint8_t> ranges_;
  std::vector<uint8_t> points_;
  bool has_points_{false};
};

}  // namespace roslean

#endif  // ROSLEAN_CORE__SCAN_POINTS_MONITOR_HPP_
