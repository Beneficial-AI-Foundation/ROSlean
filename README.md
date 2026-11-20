# ROSlean

ROS 2 nodes whose decision logic is written and proved in Lean. First target:
[`twist_mux`](https://index.ros.org/p/twist_mux/), which picks one of several
velocity command sources by priority, freshness and locks.

- [twist_mux.md](twist_mux.md): scope.
- [docs/SPEC.md](docs/SPEC.md): the rules, read from the upstream C++ source,
  and every deliberate difference.
- [docs/PLAN.md](docs/PLAN.md): development plan and status.

## Layout

| Path | Contents |
| --- | --- |
| `ROSleanCore/` | Lean, no Mathlib. The executable model (`Mux.lean`) and its C interface (`FFI.lean`). Compiled to a shared library that the node loads. |
| `ROSlean/` | Lean + Mathlib. Theorems about `ROSleanCore`. Never linked into a node. |
| `ros/roslean_core` | Builds the Lean library with lake and exposes it to ROS packages: CMake target, C++ wrapper, FFI tests. |
| `ros/roslean_twist_mux` | The node. A C++ shim with no arbitration logic: it reads parameters, subscribes, reads the clock and publishes. |
| `ros/roslean_noop` | Minimal node that calls a Lean function; the original pipeline check. |

## Quick start

Requires [elan](https://github.com/leanprover/elan) and
[pixi](https://pixi.sh). The ROS 2 Jazzy environment (RoboStack) is installed
into `.pixi/` on first use. Verified on macOS arm64 only.

```sh
pixi run build     # lake build of the core + colcon build of the ROS packages
pixi run test      # FFI tests, and scenarios compared against upstream twist_mux
pixi run prove     # check all theorems (first run fetches or builds Mathlib)
pixi run demo      # turtlesim driven through the Lean-backed mux
```

Run the node the same way as upstream:

```sh
pixi run bash -c 'source install/setup.bash && ros2 run roslean_twist_mux twist_mux \
  --ros-args --params-file ros/roslean_twist_mux/config/example.yaml'
```

## What is proved

All in `ROSlean/Mux/`, with no `sorry` and only Lean's three standard axioms.

| Property | Theorem | Statement |
| --- | --- | --- |
| Decision | `forward_iff` | A message on input *i* is forwarded iff *i* is configured with positive priority, no locked lock has higher priority, and every other live input has strictly lower priority. |
| Provenance | `forward_configured` | Only messages on configured inputs are forwarded. The message body never enters Lean, so it cannot be altered there. |
| Priority | `forward_priority` | A forwarded input outranks every other live input. |
| Lock | `forward_lock`, `not_forward_of_top_lock` | A forwarded input is not outranked by a locked lock; a lock strictly above all inputs freezes everything. |
| Dead lock publisher | `not_forward_of_silent_lock` | A lock whose publisher is silent past its timeout blocks lower inputs. |
| Expiry | `forward_of_others_expired`, `inputLive_iff_le` | An expired input never suppresses another; an input is live up to and including `stamp + timeout`. |
| Liveness | `forward_of_highest_live` | The highest live input that no lock outranks is forwarded (given positive priority). |
| Trace invariant | `run_forward_iff`, `run_forward_iff_history` | The above holds at every point of every event sequence; the decision is characterised by the event history alone. |
| Code that runs | `muxForward_after` (in `FFI.lean`) | The decision the node reads from the C interface is the model's, on exactly the handles the node configured. |

What is **not** proved: the C++ shim (mapping subscriptions to indices, reading
the clock, publishing on a `true` decision), the Lean compiler and runtime,
and ROS itself. The shim is covered by tests instead, including a comparison
with upstream `twist_mux` over the same event sequences.

Two things the theorems make precise that are easy to get wrong:

- A lock blocks only inputs of *strictly* lower priority, and priorities are
  clamped to 255. An input at 255 can never be blocked. Give an e-stop lock a
  priority above every input.
- When all inputs are silent the mux publishes nothing; it does not send a
  stop command. Stopping is the base driver's job, as with upstream.

## GitHub configuration

To set up your new GitHub repository, follow these steps:

* Under your repository name, click **Settings**.
* In the **Actions** section of the sidebar, click "General".
* Check the box **Allow GitHub Actions to create and approve pull requests**.
* Click the **Pages** section of the settings sidebar.
* In the **Source** dropdown menu, select "GitHub Actions".

After following the steps above, you can remove this section from the README file.
