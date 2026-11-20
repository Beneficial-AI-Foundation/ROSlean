// Exercises the C++ wrapper against the Lean core, without ROS. The scenarios
// repeat ROSlean/Mux/Examples.lean, so a mismatch between the declarations in
// mux.hpp and the Lean exports (argument order, widths, ownership) shows up
// as a wrong decision or a crash here.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "roslean_core/mux.hpp"

namespace
{

int failures = 0;

void check(bool cond, const std::string & what)
{
  std::printf("%s  %s\n", cond ? "ok  " : "FAIL", what.c_str());
  if (!cond) {
    ++failures;
  }
}

// Planner (priority 10) and teleop (priority 100), both expiring after 500;
// one lock with the given priority and timeout.
roslean::Mux make(uint64_t lock_priority, uint64_t lock_timeout)
{
  roslean::Mux::Builder b;
  check(b.add_input(10, 500) == 0, "planner is input 0");
  check(b.add_input(100, 500) == 1, "teleop is input 1");
  check(b.add_lock(lock_priority, lock_timeout) == 0, "lock is lock 0");
  return b.build();
}

}  // namespace

int main()
{
  roslean::initialize();
  roslean::initialize();  // idempotent

  check(roslean_noop(42) == 42, "noop");

  {
    auto m = make(255, 0);
    check(m.on_input(0, 1000), "planner drives when alone");
    check(m.on_input(1, 1050), "teleop takes over");
    check(!m.on_input(0, 1100), "planner suppressed while teleop is live");
    check(!m.on_input(0, 1550), "planner still suppressed exactly at teleop's timeout");
    check(m.on_input(0, 1551), "planner resumes strictly after teleop's timeout");
  }
  {
    auto m = make(255, 0);
    check(m.on_input(1, 1000), "teleop drives");
    m.on_lock(0, true, 1010);
    check(!m.on_input(1, 1020), "lock blocks teleop");
    check(!m.on_input(0, 1030), "lock blocks planner");
    m.on_lock(0, false, 1040);
    check(m.on_input(1, 1050), "released lock restores teleop");
  }
  {
    // Lock at teleop's priority with a timeout of 200.
    auto m = make(100, 200);
    m.on_lock(0, false, 1000);
    check(m.on_input(0, 1100), "planner drives while lock is fresh and false");
    check(!m.on_input(0, 1201), "silent lock publisher blocks planner");
    check(m.on_input(1, 1202), "equal-priority lock does not block teleop");
  }
  {
    auto m = make(255, 0);
    check(!m.on_input(7, 1000), "out-of-range input is not forwarded");
    m.on_lock(7, true, 1001);
    check(m.on_input(0, 1002), "out-of-range lock has no effect");
  }
  {
    // Argument order and width: priority and timeout must not be swapped, and
    // 64-bit times must survive.
    roslean::Mux::Builder b;
    b.add_input(1, 1000000000ULL);
    b.add_input(2, 5000000000ULL);  // > 2^32
    auto m = b.build();
    const uint64_t t0 = 1795171813000000000ULL;  // a Unix time in ns
    check(m.on_input(1, t0), "high input drives");
    check(!m.on_input(0, t0 + 5000000000ULL), "low input suppressed at 5 s");
    check(m.on_input(0, t0 + 5000000001ULL), "low input resumes after 5 s + 1 ns");
  }
  {
    roslean::Mux::Builder b;
    b.add_input(10, 0);
    b.add_input(10, 5);
    bool threw = false;
    try {
      b.build();
    } catch (const roslean::ConfigError & e) {
      threw = true;
      std::printf("      (message: %s)\n", e.what());
    }
    check(threw, "duplicate input priorities are rejected");
  }
  {
    roslean::Mux::Builder b;
    auto m = b.build();
    check(!m.on_input(0, 1), "empty mux forwards nothing");
  }
  {
    // Long run: catches reference-count mistakes (leak or double free).
    auto m = make(255, 0);
    bool all = true;
    for (uint64_t t = 1000; t < 1000000; ++t) {
      m.on_lock(0, false, t);
      all = m.on_input(1, t) && all;
    }
    check(all, "teleop forwarded throughout a long run");
  }

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
