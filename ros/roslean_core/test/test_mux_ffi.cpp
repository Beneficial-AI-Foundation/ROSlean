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
    // Equal priorities: the input added first wins while it is live (R10).
    roslean::Mux::Builder b;
    b.add_input(100, 500);
    b.add_input(100, 500);
    b.add_input(10, 500);
    auto m = b.build();
    check(m.on_input(1, 1000), "second of a tied pair drives while the first is expired");
    check(m.on_input(0, 1100), "first of a tied pair takes over at once");
    check(!m.on_input(1, 1200), "second is dropped while the first is live");
    check(!m.on_input(2, 1300), "lower input is dropped");
    check(!m.on_input(1, 1600), "second still dropped exactly at the first's timeout");
    check(m.on_input(1, 1601), "second drives 1 after the first's timeout");
  }
  {
    roslean::Mux::Builder b;
    auto m = b.build();
    check(!m.on_input(0, 1), "empty mux forwards nothing");
  }
  {
    // Values at and above 2^63 take Lean's big-number path, and
    // stamp + timeout must not wrap at 2^64.
    roslean::Mux::Builder b;
    b.add_input(1, INT64_MAX);
    b.add_input(2, 10);
    auto m = b.build();
    const uint64_t big = (1ULL << 63) + 5;
    check(m.on_input(1, big), "high input drives at t >= 2^63");
    check(!m.on_input(0, big + 10), "low input suppressed at the timeout, t >= 2^63");
    check(m.on_input(0, big + 11), "low input resumes after the timeout, t >= 2^63");
    check(m.on_input(1, UINT64_MAX - 1), "high input drives near 2^64");
    // Had stamp + timeout wrapped, the high input would look expired here.
    check(!m.on_input(0, UINT64_MAX), "stamp + timeout beyond 2^64 does not wrap");
    check(!m.on_input(UINT32_MAX, UINT64_MAX), "index UINT32_MAX is not forwarded");
  }
  {
    // Lock argument order (index, value, time) with a lock timeout at large times.
    roslean::Mux::Builder b;
    b.add_input(10, 0);
    b.add_lock(1, 0);
    b.add_lock(50, 1000);
    auto m = b.build();
    const uint64_t t0 = 1795171813000000000ULL;
    m.on_lock(1, false, t0);
    check(m.on_input(0, t0 + 1000), "fresh false lock does not block");
    check(!m.on_input(0, t0 + 1001), "lock 1 expires after its own timeout");
    m.on_lock(0, true, t0 + 2000);
    m.on_lock(1, false, t0 + 2000);
    check(m.on_input(0, t0 + 2001), "low lock (1) true does not block input (10)");
    m.on_lock(1, true, t0 + 2002);
    check(!m.on_input(0, t0 + 2003), "lock 1 true blocks");
  }
  {
    // Priorities are clamped to 255 inside Lean, which can create ties.
    roslean::Mux::Builder b;
    b.add_input(255, 0);
    b.add_input(1LL << 40, 0);
    auto m = b.build();
    check(!m.on_input(1, 1000), "2^40 clamps to 255 and loses the tie to the earlier 255");
    check(m.on_input(0, 1000), "the earlier 255 drives");
  }
  {
    // A build empties the builder.
    roslean::Mux::Builder b;
    b.add_input(5, 0);
    auto m1 = b.build();
    check(b.add_input(5, 0) == 0, "builder is empty after a build");
    auto m2 = b.build();
    check(m1.on_input(0, 1) && m2.on_input(0, 1), "two muxes from one builder are independent");

    // Moves.
    roslean::Mux m3(std::move(m1));
    check(m3.on_input(0, 2), "move-constructed mux works");
    m2 = std::move(m3);
    check(m2.on_input(0, 3), "move-assigned mux works");
    bool threw = false;
    try {
      m3.on_input(0, 4);
    } catch (const std::logic_error &) {
      threw = true;
    }
    check(threw, "moved-from mux throws instead of crashing");
  }
  {
    // Raw values are normalised in Lean: a negative priority is 0 (never
    // selected), and a negative timeout never expires.
    roslean::Mux::Builder b;
    b.add_input(-5, 500);
    b.add_input(10, 500);
    b.add_input(20, -1);
    auto m = b.build();
    check(!m.on_input(0, 1000000), "negative priority is never forwarded");
    check(!m.on_input(1, 1000000), "never-heard input with a negative timeout stays live");
    check(m.on_input(2, 1000000), "input with a negative timeout drives");
    check(!m.on_input(1, UINT64_MAX), "and never expires");
  }
  {
    roslean::Mux::Builder b;
    b.add_input(-1, 0);
    b.add_input(INT64_MIN, 0);
    auto m = b.build();
    check(!m.on_input(0, 1) && !m.on_input(1, 2), "two negative priorities: neither is selected");
  }
  {
    // Repeated build / step cycles: reference counting on every path.
    bool all = true;
    for (int n = 0; n < 20000; ++n) {
      roslean::Mux::Builder b;
      b.add_input(10, 500);
      b.add_input(10 + (n % 2), 500);
      auto m = b.build();
      // Tied (n even): the first is live at time 0 and wins. Otherwise the
      // second has the higher priority.
      all = (m.on_input(1, 400) == (n % 2 == 1)) && all;
    }
    check(all, "20000 build cycles, half of them with tied priorities");
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
