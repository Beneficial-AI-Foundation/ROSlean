# Run-time comparison with upstream twist_mux

How fast is the Lean-backed node compared with the original C++ node, and
does the difference matter on a robot?

**Short answer.** For any configuration a robot actually has (a handful of
inputs, one or two locks, commands at 10 to 1000 Hz) the two nodes are
indistinguishable in latency, loss and CPU time: the decision costs well
under a microsecond in both, against 100 to 300 µs for the trip through the
middleware. The Lean node costs about 50 MB more memory, 180 MB more disk
and 20 ms more start-up. The Lean core has one real weakness: its cost grows
with the *square* of the number of inputs, where upstream's grows linearly.
From about 64 inputs it is measurable; at 256 inputs and 50 Hz the Lean node
saturates a core and loses 40 % of the commands it should forward, while
upstream is unaffected.

Measured on 2026-11-23. Raw data: [`bench/results/`](../bench/results).

## What was measured

| Level | Lean side | Original | What it shows |
| --- | --- | --- | --- |
| Decision, synthetic time | The Lean core through `roslean::Mux`, as the node calls it | A plain C++ port of the model (`PortMux` in `bench_core.cpp`) | The cost of the Lean runtime and of the model's data structures. The port is not upstream's code. |
| Decision, node clock | The two callback bodies of `twist_mux_node.cpp` | Upstream's own sources (tag 4.5.0, compiled unchanged), called in process | The decision as each node really executes it, clock reads included. |
| Whole node | `roslean_twist_mux/twist_mux` | The `ros-jazzy-twist-mux` 4.5.0 binary | Latency through the node over real topics, loss, CPU, memory, start-up. |

Every Lean decision in the synthetic runs (about 140 million events) was compared
with the port's. They agree on all of them, including the runs with random
priorities, ties, flapping locks, expiring inputs and times beyond 2^63 ns.

Reproduce with `pixi run bench` (about 35 minutes). `bench/` is not under
`ros/`, so the normal build and the Docker image do not include it.

**Environment.** 4 virtual cores of an Intel Xeon at 2.3 GHz under KVM,
Linux 6.18, GCC 15.3, Lean 4.37.0-rc1, ROS 2 Jazzy from RoboStack with the
default middleware (Fast DDS). Lean's generated C is compiled with `-O3`.
This is a shared virtual machine, not a robot computer and not a real-time
kernel: absolute numbers will differ elsewhere, and the maxima are the
machine's scheduling, not the code (the plain C++ port, which does a few
nanoseconds of work, also shows pauses of 2 to 6 ms). Compare the two
columns with each other, and read p99.9 as the tail, not the maximum.

## Results

### The decision alone

Nanoseconds per event; every input live and speaking in turn, one lock.
"Lean" and "port" use synthetic time; "Lean, node clock" and "upstream" read
the ROS clock as the nodes do.

| Inputs | Lean | C++ port | Lean, node clock | Upstream | Lean / upstream |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 195 | 7 | 266 | 116 | 2.3 |
| 3 | 379 | 9 | 445 | 203 | 2.2 |
| 8 | 799 | 12 | 878 | 441 | 2.0 |
| 32 | 4 120 | 36 | 4 030 | 1 450 | 2.8 |
| 64 | 11 000 | 83 | | | |
| 128 | 36 300 | 159 | 36 400 | 5 850 | 6.2 |
| 256 | 138 000 | 299 | | | |
| 512 | 590 000 | 571 | 596 000 | 25 600 | 23 |
| 1024 | 2 470 000 | 781 | | | |
| 2048 (64 locks) | 13 600 000 | 2 170 | | | |

Tail at 3 inputs: Lean p50 382 ns, p99 485 ns, p99.9 876 ns. Upstream p50
221 ns, p99 248 ns, p99.9 604 ns.

The full callback (stamp, decide, publish to a matched subscriber) at 3
inputs: Lean 883 ns, upstream 679 ns. With one input, where every message is
published: 1 570 ns against 1 430 ns.

Other workloads, Lean core, nanoseconds per event:

| Workload | Lean | Port |
| --- | ---: | ---: |
| 3 inputs, 0 / 1 / 4 / 16 / 64 / 256 locks | 341 / 372 / 520 / 1 330 / 9 240 / 120 000 | 7 / 8 / 10 / 27 / 82 / 346 |
| 64 inputs; first, last or a random input speaks | 9 920 / 10 500 / 10 700 | 53 / 76 / 83 |
| 64 inputs; priorities rising, falling, equal, random | 10 700 / 10 400 / 10 400 / 10 600 | 87 / 80 / 61 / 81 |
| 8 inputs, 2 locks; timeouts off / never reached / all expired / mixed | 831 / 820 / 758 / 824 | 11 / 13 / 18 / 13 |
| the same with timeouts of 285 years (`stamp + timeout` above 2^63) | 1 500 | 13 |
| the same with clock readings above 2^63 ns | 1 290 | 13 |
| churn: 8 inputs, 4 locks, 20 % lock messages, random priorities | 814 | 16 |
| churn: 200 inputs, 50 locks, 30 % lock messages | 64 800 | 246 |
| soak: 20 million events, 4 inputs, 2 locks | 495 (p99.9 1 260) | 10 |

Memory over the soak: the Lean process ends 15.6 MB above the port's, the
same difference as in a run of 2 million events. Nothing accumulates.

### The whole node

Ten seconds of load per run, three runs per node, order alternated. Latency
is from `publish` in the load generator to the callback of a subscriber on
`cmd_vel_out`, for the input the mux should forward. In "bursts" all inputs
publish at the same instant and the winning input last.

| Scenario | Node | Forwarded | p50 µs | p99 µs | CPU % of a core | Memory (PSS) MB |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 3 inputs at 50 Hz, 1 lock | upstream | 1499 / 1500 | 299 | 2 520 | 1.7 | 21 |
| | Lean | 1500 / 1500 | 328 | 2 930 | 1.9 | 70 |
| 4 inputs at 1 kHz each | upstream | 98.0 % | 160 | 1 520 | 20.3 | 21 |
| | Lean | 98.3 % | 161 | 1 260 | 20.4 | 70 |
| 1 input at 5 kHz (2 silent) | upstream | 98.5 % | 99 | 250 | 33.7 | 21 |
| | Lean | 98.7 % | 99 | 258 | 34.9 | 70 |
| 16 inputs at 100 Hz, bursts | upstream | 2999 / 3000 | 288 | 2 220 | 3.1 | 23 |
| | Lean | 3000 / 3000 | 314 | 2 070 | 3.4 | 72 |
| 64 inputs at 50 Hz, bursts | upstream | 1500 / 1500 | 483 | 3 580 | 4.8 | 27 |
| | Lean | 1499 / 1500 | 756 | 3 480 | 7.8 | 76 |
| 256 inputs at 10 Hz, bursts | upstream | 300 / 300 | 2 430 | 14 700 | 6.5 | 44 |
| | Lean | 300 / 300 | 31 000 | 52 400 | 35.4 | 93 |
| 256 inputs at 50 Hz, bursts | upstream | 1498 / 1500 | 3 410 | 8 920 | 28.4 | 44 |
| | **Lean** | **876 / 1500** | 16 500 | 20 800 | **92.7** | 93 |
| 512 inputs at 2 Hz, bursts | upstream | 60 / 60 | 11 100 | 21 400 | 4.8 | 68 |
| | Lean | 60 / 60 | 191 000 | 248 000 | 55.3 | 116 |

Neither node forwarded a message from a wrong input in any run but the
last. There both did (38 and 12 messages): the period equals the 0.5 s input
timeout in that scenario, so the winner is sometimes expired when the next
burst starts. That is the configuration, not either node.

The 1 to 2 % missing at 1 kHz and 5 kHz are lost between the load generator
and the mux at these rates on this machine, equally for both nodes.

Start-up (process start until all subscriptions are discovered, 3 inputs,
median of 10): upstream 26 ms, Lean 47 ms. On disk, the Lean node needs
`libleanshared.so`, 183 MB on Linux; upstream's binary is under 1 MB.

## Where the differences come from

**Constant cost: about 0.2 µs more per event.** Upstream's decision is a
loop over a `std::list` with one clock read per handle (about 45 ns each;
with all timeouts off it drops from 441 ns to 75 ns at 8 inputs) and a
string comparison. Lean's reads the clock once, then pays for boxed
values, reference counting and allocation: each event builds a fresh index
list for the inputs and for the locks and a pair per candidate, and frees
them. The plain port shows what the rule itself costs: 9 ns.

**Growth: quadratic in Lean, linear upstream.** The model keeps
configuration and state in linked lists and `select` looks each input up by
index inside the loop over the inputs (`cfg.inputs[k]?` twice and
`st.inputStamps.getD k` once per input in `unmasked` and `inputLive`); locks
likewise in `lockPriority`. Doubling the inputs quadruples the time: 138 µs
at 256, 590 µs at 512, 2.5 ms at 1024. Which input speaks and how priorities
are laid out make no difference; the lookups dominate.

This is the model's choice of data structure, not a property of Lean. The
model was written for the proofs, where lists and index lookups are
convenient. As a check, the same rule written over arrays in one pass
([`bench/experiments/ArrayMux.lean`](../bench/experiments/ArrayMux.lean),
not proved, not used) runs in 62 ns at 3 inputs, 2.2 µs at 256 and 12 µs at
2048: linear, and faster than upstream as measured here. Getting that into
the node means either re-proving the property table for the array version or
proving it equal to the current `step` and keeping the theorems as they are.

**No pauses.** Lean frees memory by reference counting; there is no
collector that stops the program. The tail is flat: p99 stays within 1.5
times the median, and p99.9 within 2.5 times at 3 or 4 inputs. From 8 inputs
up p99.9 is about 14 µs, which upstream shows as well once its decision is
as long (32 inputs); it is the machine's timer interrupt landing in a longer
measurement. 20 million events show no growth in memory or latency.

**Unbounded numbers cost time only when they are needed.** Lean's naturals
never overflow. Sums that fit 63 bits are machine arithmetic; beyond that
Lean switches to heap-allocated big numbers, which nearly doubles the cost
(820 ns to 1 500 ns at 8 inputs). That takes a timeout above about 235 years
or a clock beyond the year 2262, so it does not occur, and the decisions
stay correct when it does. Upstream computes in double-precision seconds,
which at today's epoch resolves about 240 ns (SPEC, D2).

**Memory.** The 50 MB are the Lean runtime and standard library: about
48 MB of private memory that the process touches when the shared library is
loaded and initialised, whatever the configuration. The mux state itself is
a few hundred bytes.

## What this means on a robot

- **Normal use: no difference you can measure from outside.** A mobile base
  has two to six command sources at 10 to 100 Hz. The Lean decision takes
  0.4 µs, upstream's 0.2 µs, and the message spends 100 to 300 µs in the
  middleware either way. Against a 20 ms control period the extra 0.2 µs is
  one part in 100 000. At 4 kHz total and at a 5 kHz flood the two nodes have
  the same latency, loss and CPU use. The proofs cost nothing at run time
  here.

- **The e-stop path is as fast as upstream's.** A lock message is a state
  update (no decision), and the next command on any input sees it. Nothing in
  the Lean core delays that by more than its 0.4 µs.

- **Do not use the current core with hundreds of inputs.** 16 inputs are
  fine (the median latency is 9 % above upstream's, within the spread
  between runs). At 64 the node uses 60 % more
  CPU and adds 0.3 ms when all inputs fire together. At 256 a burst takes
  36 ms to work through, so the winning command arrives 30 ms late at 10 Hz,
  and at 50 Hz the node cannot keep up: 93 % of a core, 40 % of the winning
  commands lost in the subscription queues. Such configurations are far from
  usual for `twist_mux`, but a generated configuration (a fleet gateway, one
  mux for many teleoperation clients) could reach them, and the failure is a
  cliff, not a slope. Nothing warns of it at start-up.

- **A slow mux is worse than slow: it misjudges freshness.** Both nodes
  stamp a message when its callback runs, not when it arrived. When the node
  falls behind, a command that waited 200 ms in a queue is stamped as new and
  can keep its input "live" past its timeout, and commands are forwarded
  late or dropped unseen. The theorems hold for the events the core was
  given; they say nothing about events it has not got to yet. This applies
  to upstream as well, but only the Lean node reaches that state at these
  sizes.

- **Memory and disk matter on small computers only.** 70 MB resident and
  183 MB of library are nothing on an industrial PC or a Jetson, noticeable
  on a 512 MB single-board computer that runs several such nodes (each
  process pays the 48 MB again; nodes sharing one process would pay it once),
  and rule out microcontrollers (micro-ROS) entirely.

- **Start-up is 20 ms longer.** Irrelevant at boot. It also applies when a
  supervisor restarts a crashed mux, where it is still small beside DDS
  discovery.

- **Hard real time: neither node qualifies, for the same reasons.** Both
  allocate on the heap in the callback (Lean for its lists, upstream for a
  message copy and a `std::string` per decision) and both run in the
  standard `rclcpp` executor over DDS. Lean's allocation is the more
  frequent. With a real-time allocator and executor the Lean core would
  need to stop allocating per event, which the array version would nearly
  achieve. Not measured here: a `PREEMPT_RT` kernel, ARM hardware, other
  middlewares, simulated time.

- **One thread.** The Lean object must not be used from two threads at once,
  and the node uses a single-threaded executor. Upstream is used the same
  way in practice. There is nothing to gain from more threads at these
  costs.

## Recommendation

Use the node as it is for ordinary configurations; the run-time price of the
verified core is 50 MB of memory. Before anyone relies on it with more than
a few dozen inputs, replace the lists in the executable model with arrays in
a single pass and carry the proofs over. Until then, a warning at start-up
above about 32 inputs or locks would make the limit visible.

## Limits of this comparison

- One machine, virtualised, not isolated from other tenants. Node-level p99
  and maxima vary by a factor of two between identical runs.
- CPU and memory were read from `/proc`; CPU includes the node's start-up
  after its first 0.2 s.
- The "port" is this repository's reading of the rule in C++, not upstream.
  Upstream's own code appears only in the node-clock and whole-node rows.
- Upstream also publishes diagnostics once a second, which the Lean node
  does not do; that work is in upstream's figures.
- Start-up of upstream with 512 inputs was 1.2 s in two runs out of three
  and 0.2 s in the third; the cause was not investigated.
