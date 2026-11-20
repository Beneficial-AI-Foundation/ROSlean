/-!
# No-op core

The smallest possible Lean "core" for a ROS node: a function exported to C that
returns its argument unchanged. It exists only to prove out the Lean → C → ROS
build and link pipeline; the real `step` function replaces it.

This library must stay free of Mathlib (and of anything else that is expensive
to link): it is compiled to native code and loaded into the ROS node. Proofs
about it live in the `ROSlean` library, which may import Mathlib.
-/

namespace ROSleanCore

/-- Identity on a sequence number. The ROS node calls this once per tick. -/
@[export roslean_noop]
def noop (seq : UInt64) : UInt64 := seq

end ROSleanCore
