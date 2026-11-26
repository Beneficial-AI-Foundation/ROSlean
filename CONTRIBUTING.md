# Contributing to ROSlean

Thanks for your interest in contributing. ROSlean builds ROS 2 nodes whose
decision logic is written and proved in Lean 4; the first target is a
`twist_mux`-compatible velocity multiplexer. This guide covers how to build it,
the standards we hold the proofs to, and how to send a change. It follows the
shared contribution standard used across the
[Beneficial AI Foundation](https://github.com/Beneficial-AI-Foundation) Lean
projects.

This repository is in an exploratory, still unstable phase — expect interfaces
to move.

## Prerequisites

- [`elan`](https://github.com/leanprover/elan), the Lean toolchain manager. It
  reads `lean-toolchain` and fetches the exact Lean/Lake version this project
  uses, so you don't install Lean by hand:

  ```sh
  curl https://raw.githubusercontent.com/leanprover/elan/master/elan-init.sh -sSf | sh
  ```

- [`pixi`](https://pixi.sh), used to build and test the ROS 2 side. On first use
  it installs a ROS 2 Jazzy environment (RoboStack, about 1 GB) into `.pixi/`.

Only `elan` is needed to build and check the Lean library and its proofs; `pixi`
(and, through it, ROS 2) is needed to build the node, run the comparison tests
and run the demos.

## Building

```sh
# The Lean side (no ROS, no pixi needed):
lake build            # the Mathlib-free executable core (ROSleanCore)
lake exe cache get    # Mathlib's build cache (first proof build only)
lake build ROSlean    # the Mathlib-backed proofs about the core

# The ROS 2 side (needs pixi / ROS 2 Jazzy):
pixi run build        # colcon build of the core + the ROS packages
```

## Testing

```sh
pixi run prove        # check all theorems (fetches or builds Mathlib first)
pixi run test         # FFI tests and scenarios compared against upstream twist_mux
pixi run demo         # turtlesim driven through the Lean-backed mux
```

Continuous integration (`.github/workflows/ci.yml`) builds the executable core
and the Mathlib-backed proofs on Linux and macOS for every push and pull
request — the equivalent of `pixi run prove`. It does **not** run the ROS 2
integration (`pixi run test`, the C++ shim, the comparison with upstream
`twist_mux`): that needs a full ROS 2 Jazzy install, which CI does not stand up.
Please run `pixi run test` locally before opening a PR that touches the node.

## Proof discipline

Correctness here comes from machine-checked proofs, so the sources must stay free
of proof escape hatches:

- **No `sorry` or `admit`.** The theorems in `ROSlean/Mux/` are complete.
- The proofs rely only on Lean's three standard axioms — `propext`,
  `Classical.choice` and `Quot.sound`. Check with `#print axioms <theorem>` if
  you are unsure.
- The executable core (`core/ROSleanCore/`) stays free of Mathlib, so building a
  ROS node never needs it; keep new runtime code there and new proofs in
  `ROSlean/`.

## Submitting a change

1. **Branch** off `main` (or fork the repository).
2. Keep each pull request focused, and add or update a spec/theorem for
   behaviour you change — tests are theorems here.
3. Make sure `lake build ROSlean` (and `pixi run test`, when you touch the node)
   pass, and that CI is green.
4. Write a clear PR description: what changed, why, and how you verified it.
5. By contributing, you agree your contribution is licensed under this
   repository's license (see `LICENSE` and `NOTICE`), and you certify the
   [Developer Certificate of Origin](https://developercertificate.org/) — add a
   `Signed-off-by: Your Name <you@example.com>` line to your commits
   (`git commit -s`).

## Reporting issues

Open a GitHub issue with a clear title, what you expected, what happened, and the
smallest configuration, request or scenario that reproduces it. Security-sensitive
reports can be sent privately to **support@beneficialaifoundation.org** instead
of a public issue.

## Code of conduct

Be respectful and constructive. Harassment or abuse is not tolerated. Maintainers
may remove comments, commits or contributions that don't meet this standard.
