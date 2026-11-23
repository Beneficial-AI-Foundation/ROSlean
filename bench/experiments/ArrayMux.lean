/-! Experiment only, for docs/BENCHMARK.md: the decision rule of
`ROSleanCore.Mux.step` over arrays, in one pass, with the state updated in
place. Not proved, not compared with the model, and not used by any node.

To run it, make this file the root of a `lean_exe` in a scratch lake package.
-/
structure H where
  priority : Nat
  timeout : Nat
  deriving Inhabited

structure M where
  inputs : Array H
  locks : Array H
  inS : Array Nat
  lkS : Array Nat
  lkV : Array Bool

@[inline] def expired (timeout stamp now : Nat) : Bool := 0 < timeout && stamp + timeout < now

@[noinline] def onInput (m : M) (i : Nat) (t : Nat) : M × Bool :=
  let ⟨inputs, locks, inS, lkS, lkV⟩ := m
  if i < inputs.size then
    let inS := inS.set! i t
    let lp := Id.run do
      let mut lp := 0
      for j in [0:locks.size] do
        let h := locks[j]!
        if (expired h.timeout lkS[j]! t || lkV[j]!) && lp < h.priority then lp := h.priority
      return lp
    let sel := Id.run do
      let mut best := 0
      let mut sel := inputs.size
      for k in [0:inputs.size] do
        let h := inputs[k]!
        if !expired h.timeout inS[k]! t && lp ≤ h.priority && best < h.priority then
          best := h.priority
          sel := k
      return sel
    (⟨inputs, locks, inS, lkS, lkV⟩, sel == i)
  else (⟨inputs, locks, inS, lkS, lkV⟩, false)

def bench (n events : Nat) : IO Unit := do
  let inputs := (Array.range n).map fun i => ({ priority := 1 + i * 254 / (max (n - 1) 1), timeout := 1000000000000000 } : H)
  let mut m : M := { inputs, locks := #[⟨50, 1000000000000000⟩], inS := Array.replicate n 0, lkS := #[0], lkV := #[false] }
  let mut t := 1800000000000000000
  let mut fwd := 0
  let t0 ← IO.monoNanosNow
  for k in [0:events] do
    t := t + 1000000
    let (m', f) := onInput m (k % n) t
    m := m'
    if f then fwd := fwd + 1
  let t1 ← IO.monoNanosNow
  IO.println s!"n={n} events={events} forwarded={fwd} mean_ns={(t1 - t0).toFloat / events.toFloat}"

def main : IO Unit := do
  for n in [1, 3, 8, 32, 64, 256, 1024, 2048] do
    bench n (max 2000 (20000000 / (n + 5)))
