import ROSlean.CollisionMonitor.Lifecycle

/-!
# The collision monitor over event sequences

Lifts the single-step theorems of `ROSlean.CollisionMonitor.Properties` and
`ROSlean.CollisionMonitor.Lifecycle` to `run`. The node starts in
`State.init cfg`, which is unconfigured: every statement "from start-up" is
about an arbitrary list of events from that state, lifecycle transitions
included, in any order (a transition the lifecycle does not allow is a
no-op).

* `run_getElem?` is the trace invariant: the `n`-th output of any run is the
  single-step output for the `n`-th event in the state reached after the
  first `n` events. Every single-step theorem therefore holds at every point
  of every run.
* `run_phase`, `run_scan` tie the state to the history: the phase is the
  fold of `nextPhase` over the events, and the stored scan is the one of the
  scan event received last while the node was inactive or active, unless a
  cleanup came after it (`lastScan`). `reachable_inv`: every reachable state
  satisfies the invariants `Inv`.
* **Inactive** (`run_silent_before_activate`, `run_silent_after_shutdown`,
  `run_silent_after_deactivate`): no output before the first `activate`,
  none after a `shutdown`, none between a `deactivate` and the next
  `activate`.
* The properties of the SPEC for a command that arrives after an arbitrary
  history (`run_stop`, `run_stale`, `run_no_transform`, `run_pass_through`,
  `run_slowdown_bound`, `run_no_amplification`).
* **No shape** (`run_no_shape`, `run_output_of_no_shape`) and **After
  activation** (`run_after_activation`).
* `stop_pub_timeout` (R15) is the one place where an active node withholds a
  velocity while a stop is demanded. `run_withheld_last_published_zero` says
  what makes that acceptable: whenever a processed command gets no velocity,
  the velocity published last before it was a zero. `run_zero_streak` and
  `zero_period` say what `prevZero` and `stopStamp` are during a period of
  zero and exactly when its zeros are published.

Nothing is assumed about the clock (A2).
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## `run` -/

theorem run_nil (cfg : Config) (st : State) : run cfg st [] = (st, []) := rfl

theorem run_cons (cfg : Config) (st : State) (e : Event) (es : List Event) :
    run cfg st (e :: es) =
      ((run cfg (step cfg st e).1 es).1,
        (step cfg st e).2 :: (run cfg (step cfg st e).1 es).2) := by
  cases h : step cfg st e with
  | mk st' out =>
    cases h2 : run cfg st' es with
    | mk st'' outs => simp [run, h, h2]

/-- Running two sequences one after the other. -/
theorem run_append (cfg : Config) (st : State) (es es' : List Event) :
    run cfg st (es ++ es') =
      ((run cfg (run cfg st es).1 es').1,
        (run cfg st es).2 ++ (run cfg (run cfg st es).1 es').2) := by
  induction es generalizing st with
  | nil => simp [run_nil]
  | cons e es ih => simp [run_cons, ih]

/-- One more event: the new state and the new output are those of `step` in
the state reached so far. -/
theorem run_concat (cfg : Config) (st : State) (es : List Event) (e : Event) :
    run cfg st (es ++ [e]) =
      ((step cfg (run cfg st es).1 e).1,
        (run cfg st es).2 ++ [(step cfg (run cfg st es).1 e).2]) := by
  rw [run_append, run_cons, run_nil]

/-- One output per event. -/
theorem run_length (cfg : Config) (st : State) (es : List Event) :
    (run cfg st es).2.length = es.length := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih => simp [run_cons, ih]

/-- **Trace invariant.** The `n`-th output of a run is the single-step output
for the `n`-th event in the state reached after the first `n` events. -/
theorem run_getElem? (cfg : Config) (st : State) (es : List Event) (n : Nat) :
    (run cfg st es).2[n]? =
      es[n]?.map (fun e => (step cfg (run cfg st (es.take n)).1 e).2) := by
  induction es generalizing st n with
  | nil => simp [run_nil]
  | cons e es ih =>
    cases n with
    | zero => simp [run_cons, run_nil]
    | succ n => simp [run_cons, ih]

/-- The `n`-th output, given the `n`-th event. -/
theorem run_output (cfg : Config) (st : State) (es : List Event) (n : Nat) (e : Event)
    (h : es[n]? = some e) :
    (run cfg st es).2[n]? = some (step cfg (run cfg st (es.take n)).1 e).2 := by
  rw [run_getElem?, h]; rfl

/-- The state after the first `n + 1` events is one step from the state after
the first `n`. -/
theorem run_take_succ (cfg : Config) (st : State) (es : List Event) (n : Nat) (e : Event)
    (h : es[n]? = some e) :
    (run cfg st (es.take (n + 1))).1 = (step cfg (run cfg st (es.take n)).1 e).1 := by
  rw [List.take_add_one, h, Option.toList_some, run_concat]

/-- **Only processed commands are answered.** If position `n` of a run has an
output other than `silent`, the event there is a finite command and the node
was active when it arrived. -/
theorem run_processed (cfg : Config) (st : State) (es : List Event) (n : Nat) (o : Output)
    (h : (run cfg st es).2[n]? = some o) (ho : o ≠ .silent) :
    ∃ now v scanPts zoneVerts, es[n]? = some (.cmd now v scanPts zoneVerts) ∧
      (run cfg st (es.take n)).1.phase = .active ∧ v.finite = true := by
  rw [run_getElem?] at h
  cases he : es[n]? with
  | none => simp [he] at h
  | some e =>
    rw [he] at h
    simp only [Option.map_some, Option.some.injEq] at h
    subst h
    obtain ⟨now, v, scanPts, zoneVerts, rfl, hp, hf⟩ := (processed_iff _ _ _).mp ho
    exact ⟨now, v, scanPts, zoneVerts, rfl, hp, hf⟩

/-! ## The phase, the invariants, the stored scan -/

/-- The phase after a sequence of events. -/
def phaseAfter (p : Phase) (es : List Event) : Phase := es.foldl nextPhase p

theorem phaseAfter_cons (p : Phase) (e : Event) (es : List Event) :
    phaseAfter p (e :: es) = phaseAfter (nextPhase p e) es := rfl

/-- The phase after a run is a function of the phase before it and the
lifecycle events in it. -/
theorem run_phase (cfg : Config) (st : State) (es : List Event) :
    (run cfg st es).1.phase = phaseAfter st.phase es := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih => rw [run_cons, ih, step_phase, phaseAfter_cons]

/-- The invariants hold after any run from a state in which they hold. -/
theorem run_inv (cfg : Config) (st : State) (es : List Event) (h : Inv cfg st) :
    Inv cfg (run cfg st es).1 := by
  induction es generalizing st with
  | nil => exact h
  | cons e es ih => rw [run_cons]; exact ih _ (inv_step cfg st e h)

/-- **Lifecycle invariants.** Every state reachable from start-up satisfies
`Inv`. -/
theorem reachable_inv (cfg : Config) (es : List Event) : Inv cfg (run cfg (State.init cfg) es).1 :=
  run_inv cfg _ es (inv_init cfg)

/-- Once finalized, always finalized. -/
theorem phaseAfter_finalized (es : List Event) : phaseAfter .finalized es = .finalized := by
  induction es with
  | nil => rfl
  | cons e es ih =>
    rw [phaseAfter_cons]
    have : nextPhase .finalized e = .finalized := by cases e <;> simp [nextPhase]
    rw [this, ih]

/-- Without an `activate`, a node that is not active does not become
active. -/
theorem phaseAfter_ne_active (p : Phase) (es : List Event) (hp : p ≠ .active)
    (h : ∀ e ∈ es, e ≠ .activate) : phaseAfter p es ≠ .active := by
  induction es generalizing p with
  | nil => exact hp
  | cons e es ih =>
    rw [phaseAfter_cons]
    apply ih
    · have he := h e List.mem_cons_self
      cases e <;> simp only [nextPhase] <;> first | exact hp | (split <;> simp_all) | simp_all
    · exact fun e' he' => h e' (List.mem_cons_of_mem _ he')

/-- The scan in force after one event: a scan event replaces it while the
node is inactive or active (N4), a cleanup in the inactive phase discards it
(N7), nothing else touches it. -/
def scanAfter (p : Phase) (d : Option Scan) : Event → Option Scan
  | .scan _ s => if p = .inactive ∨ p = .active then some s else d
  | .cleanup => if p = .inactive then none else d
  | _ => d

/-- The scan in force after the sequence, from phase `p` and scan `d`: that
of the scan event received last while the node was inactive or active,
unless a cleanup followed it. The last *received*, whatever the stamps
say. -/
def lastScan : List Event → Phase → Option Scan → Option Scan
  | [], _, d => d
  | e :: es, p, d => lastScan es (nextPhase p e) (scanAfter p d e)

theorem step_scan_field (cfg : Config) (st : State) (e : Event) :
    (step cfg st e).1.scan = scanAfter st.phase st.scan e := by
  cases e with
  | cmd now v scanPts zoneVerts => exact (step_cmd_scan cfg st now v scanPts zoneVerts).1
  | shape k verts foreign =>
    simp only [step, scanAfter]
    split <;> rfl
  | scan now s =>
    simp only [step, scanAfter]
    split <;> rfl
  | cleanup =>
    simp only [step, scanAfter]
    split <;> rfl
  | shutdown => rfl
  | _ =>
    simp only [step, scanAfter]
    split <;> rfl

/-- After any run, over any events, the stored scan is the one in force.
Commands and shape messages do not touch it. -/
theorem run_scan (cfg : Config) (st : State) (es : List Event) :
    (run cfg st es).1.scan = lastScan es st.phase st.scan := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih => rw [run_cons, ih, step_phase, step_scan_field]; rfl

theorem lastScan_append (es es' : List Event) (p : Phase) (d : Option Scan) :
    lastScan (es ++ es') p d = lastScan es' (phaseAfter p es) (lastScan es p d) := by
  induction es generalizing p d with
  | nil => rfl
  | cons e es ih => simp only [List.cons_append, lastScan, ih, phaseAfter_cons]

/-- A scan that arrives while the node is inactive or active is the one in
force afterwards, whatever came before. -/
theorem lastScan_append_scan (es : List Event) (now : Nat) (s : Scan) (p : Phase)
    (d : Option Scan) (h : phaseAfter p es = .inactive ∨ phaseAfter p es = .active) :
    lastScan (es ++ [.scan now s]) p d = some s := by
  rw [lastScan_append]
  simp [lastScan, scanAfter, h]

/-- Commands and shape messages do not change the scan in force. -/
theorem lastScan_append_cmd (es : List Event) (now : Nat) (v : Twist)
    (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords))) (p : Phase) (d : Option Scan) :
    lastScan (es ++ [.cmd now v scanPts zoneVerts]) p d = lastScan es p d := by
  rw [lastScan_append]
  rfl

/-- The points tested after any events since start-up, from the history
alone (R5 to R8, N27). -/
theorem run_sourcePoints_eq_some_iff (cfg : Config) (es : List Event) (now : Nat)
    (scanPts : Option (List Coords)) (pts : List Point) :
    sourcePoints cfg (run cfg (State.init cfg) es).1 now scanPts = some pts ↔
      (cfg.sourceEnabled = false ∧ pts = []) ∨
      (cfg.sourceEnabled = true ∧ lastScan es .unconfigured none = none ∧
        cfg.sourceTimeout = 0 ∧ pts = []) ∨
      (cfg.sourceEnabled = true ∧ ∃ s, lastScan es .unconfigured none = some s ∧
        ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now) ∧
        ((∃ c, scanPts = some c ∧ s.fits c = true ∧ pts = s.points c) ∨
          (NoScanPoints (run cfg (State.init cfg) es).1 scanPts ∧ cfg.sourceTimeout = 0 ∧
            pts = []))) := by
  rw [sourcePoints_eq_some_iff, run_scan]
  rfl

/-! ## Inactive: when nothing is published -/

/-- **Inactive, over a run.** From a state that is not active, over events
without an `activate`, every output is `silent` and the node stays not
active. -/
theorem run_silent_of_no_activate (cfg : Config) (st : State) (es : List Event)
    (hp : st.phase ≠ .active) (h : ∀ e ∈ es, e ≠ .activate) :
    (run cfg st es).1.phase ≠ .active ∧ ∀ o ∈ (run cfg st es).2, o = .silent := by
  refine ⟨by rw [run_phase]; exact phaseAfter_ne_active _ _ hp h, ?_⟩
  intro o ho
  obtain ⟨n, hn⟩ := List.mem_iff_getElem?.mp ho
  by_contra hne
  obtain ⟨now, v, scanPts, zoneVerts, he, hact, -⟩ := run_processed cfg st es n o hn hne
  rw [run_phase] at hact
  exact phaseAfter_ne_active _ _ hp
    (fun e' he' => h e' (List.mem_of_mem_take he')) hact

/-- **No output before the first `activate`.** In any run from start-up, if
no event before position `n` is an `activate`, the output at position `n` is
`silent`. -/
theorem run_silent_before_activate (cfg : Config) (es : List Event) (n : Nat) (e : Event)
    (he : es[n]? = some e) (hno : ∀ m, m < n → es[m]? ≠ some .activate) :
    (run cfg (State.init cfg) es).2[n]? = some .silent := by
  rw [run_output _ _ _ _ _ he]
  congr 1
  apply output_of_not_active
  rw [run_phase]
  apply phaseAfter_ne_active
  · simp [State.init]
  · intro e' he' hc
    obtain ⟨m, hm⟩ := List.mem_iff_getElem?.mp he'
    rw [List.getElem?_take] at hm
    split at hm
    · rename_i hlt
      exact hno m hlt (by rw [hm, hc])
    · cases hm

/-- The phase after the first `n` events, if one of them is a `shutdown`. -/
theorem phaseAfter_take_of_shutdown (p : Phase) (es : List Event) (m n : Nat) (hmn : m < n)
    (h : es[m]? = some .shutdown) : phaseAfter p (es.take n) = .finalized := by
  induction es generalizing p m n with
  | nil => simp at h
  | cons e es ih =>
    cases n with
    | zero => omega
    | succ n =>
      rw [List.take_succ_cons, phaseAfter_cons]
      cases m with
      | zero =>
        simp only [List.getElem?_cons_zero, Option.some.injEq] at h
        subst h
        exact phaseAfter_finalized _
      | succ m => exact ih _ m n (by omega) (by simpa using h)

/-- **No output after a `shutdown`.** In any run, from any state, every
output after a `shutdown` event is `silent`, whatever follows it (ND4). -/
theorem run_silent_after_shutdown (cfg : Config) (st : State) (es : List Event) (m n : Nat)
    (e : Event) (hmn : m < n) (hm : es[m]? = some .shutdown) (he : es[n]? = some e) :
    (run cfg st es).2[n]? = some .silent := by
  rw [run_output _ _ _ _ _ he]
  congr 1
  apply output_of_not_active
  rw [run_phase, phaseAfter_take_of_shutdown _ es m n hmn hm]
  simp

/-- The phase after the first `n` events, if one of them is a `deactivate`
and no `activate` follows it. -/
theorem phaseAfter_take_of_deactivate (p : Phase) (es : List Event) (m n : Nat) (hmn : m < n)
    (h : es[m]? = some .deactivate) (hno : ∀ j, m < j → j < n → es[j]? ≠ some .activate) :
    phaseAfter p (es.take n) ≠ .active := by
  induction es generalizing p m n with
  | nil => simp at h
  | cons e es ih =>
    cases n with
    | zero => omega
    | succ n =>
      rw [List.take_succ_cons, phaseAfter_cons]
      cases m with
      | zero =>
        simp only [List.getElem?_cons_zero, Option.some.injEq] at h
        subst h
        apply phaseAfter_ne_active
        · simp only [nextPhase]; split <;> simp_all
        · intro e' he' hc
          obtain ⟨j, hj⟩ := List.mem_iff_getElem?.mp he'
          rw [List.getElem?_take] at hj
          split at hj
          · rename_i hlt
            exact hno (j + 1) (by omega) (by omega) (by simpa [hc] using hj)
          · cases hj
      | succ m =>
        exact ih _ m n (by omega) (by simpa using h)
          (fun j hj1 hj2 => by simpa using hno (j + 1) (by omega) (by omega))

/-- **No output between a `deactivate` and the next `activate`.** In any run,
from any state, if position `m` is a `deactivate` and no `activate` lies
strictly between `m` and `n`, the output at position `n` is `silent`. -/
theorem run_silent_after_deactivate (cfg : Config) (st : State) (es : List Event) (m n : Nat)
    (e : Event) (hmn : m < n) (hm : es[m]? = some .deactivate) (he : es[n]? = some e)
    (hno : ∀ j, m < j → j < n → es[j]? ≠ some .activate) :
    (run cfg st es).2[n]? = some .silent := by
  rw [run_output _ _ _ _ _ he]
  congr 1
  apply output_of_not_active
  rw [run_phase]
  exact phaseAfter_take_of_deactivate _ es m n hmn hm hno

/-! ## The properties of the SPEC, after any events since start-up

In each statement `es` is an arbitrary sequence of events of all kinds, and
the command `(now, v, scanPts, zoneVerts)` arrives next. `phaseAfter
.unconfigured es` is the phase reached, `lastScan es .unconfigured none` the
scan in force. By `run_concat` the command's output is the last element of
`(run cfg (State.init cfg) (es ++ [.cmd now v scanPts zoneVerts])).2`.

A zone named in a hypothesis is a zone with a fixed shape: such a zone is in
force in every state (`liveConfig_of_fixed`). For topic zones the
single-step theorems apply at every point of a run, with the shape the state
holds. -/

/-- A zone with a fixed shape stands in the zones in force as it is
configured, in every state. -/
theorem liveConfig_of_fixed (cfg : Config) (st : State) (zoneVerts : List (Option (List Coords)))
    (k : Nat)
    (z : Zone) (s : Shape) (hz : cfg.zones[k]? = some z) (hs : z.shape = .fixed s) :
    (liveConfig cfg st zoneVerts).zones[k]? = some z := by
  unfold liveConfig
  rw [withShapes_getElem?, hz]
  simp [withShape_of_fixed z _ s hs]

/-- **Stop, over a run.** After any events since start-up that leave the node
active: if the source is enabled, the scan in force is not too old, the
command carries coordinates `c` that fit it, and at least `min_points` of
the scan's points under `c` are in an enabled stop zone with a fixed shape,
then the velocity published for a finite command is `stopZone k` for the
first triggered stop zone, unless zero has already been demanded for more
than `stop_pub_timeout` (see `run_withheld_last_published_zero` for what was
published then). -/
theorem run_stop (cfg : Config) (es : List Event) (now : Nat) (v : Twist) (c : List Coords)
    (zoneVerts : List (Option (List Coords))) (s : Scan) (j : Nat) (z : Zone) (sh : Shape)
    (hact : phaseAfter .unconfigured es = .active) (hf : v.finite = true)
    (he : cfg.sourceEnabled = true) (hlast : lastScan es .unconfigured none = some s)
    (hfresh : ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now))
    (hu : s.fits c = true) (hz : cfg.zones[j]? = some z) (hfix : z.shape = .fixed sh)
    (hs : z.action = .stop) (ht : z.triggered (s.points c) = true) :
    ∃ k, IsFirstStop (liveConfig cfg (run cfg (State.init cfg) es).1 zoneVerts).zones
        (s.points c) k ∧
      (step cfg (run cfg (State.init cfg) es).1 (.cmd now v (some c) zoneVerts)).2.velocity =
        if (run cfg (State.init cfg) es).1.prevZero = true ∧
          (run cfg (State.init cfg) es).1.stopStamp + cfg.stopPubTimeout < now
        then none else some (.stopZone k) := by
  have hpts : sourcePoints cfg (run cfg (State.init cfg) es).1 now (some c) = some (s.points c) :=
    (run_sourcePoints_eq_some_iff cfg es now _ _).mpr
      (Or.inr (Or.inr ⟨he, s, hlast, hfresh, Or.inl ⟨c, rfl, hu, rfl⟩⟩))
  have hp : (run cfg (State.init cfg) es).1.phase = .active := by rw [run_phase]; exact hact
  obtain ⟨k, hk, -, hout⟩ := stop cfg _ now v (some c) zoneVerts (s.points c) z hp hf hpts
    (List.mem_of_getElem? (liveConfig_of_fixed cfg _ zoneVerts j z sh hz hfix)) hs ht
  exact ⟨k, hk, hout⟩

/-- **Stale data, over a run.** After any events since start-up that leave
the node active: if the source is enabled with a non-zero timeout, and no
scan is in force (none has arrived since the last cleanup) or the stamp of
the scan in force is more than the timeout before `now`, the velocity
published for a finite command is `stopSource`, whatever coordinates the
command carries, unless zero has already been demanded for more than
`stop_pub_timeout`. -/
theorem run_stale (cfg : Config) (es : List Event) (now : Nat) (v : Twist)
    (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords)))
    (hact : phaseAfter .unconfigured es = .active) (hf : v.finite = true)
    (he : cfg.sourceEnabled = true) (ht : 0 < cfg.sourceTimeout)
    (hs : lastScan es .unconfigured none = none ∨
      ∃ s, lastScan es .unconfigured none = some s ∧ s.stamp + cfg.sourceTimeout < now) :
    (step cfg (run cfg (State.init cfg) es).1 (.cmd now v scanPts zoneVerts)).2.velocity =
      if (run cfg (State.init cfg) es).1.prevZero = true ∧
        (run cfg (State.init cfg) es).1.stopStamp + cfg.stopPubTimeout < now
      then none else some .stopSource := by
  refine (stale cfg _ now v scanPts zoneVerts (by rw [run_phase]; exact hact) hf he ht ?_).2
  rw [run_scan]
  exact hs

/-- **No transform, over a run.** After any events since start-up that leave
the node active: if the source is enabled with a non-zero timeout and the
command carries no coordinates that fit the scan in force (none at all, when
the transform lookup failed), the velocity published for a finite command is
`stopSource`, whatever scan is in force, unless zero has already been
demanded for more than `stop_pub_timeout`. This repeats at every command
until one carries the scan's points (N22). -/
theorem run_no_transform (cfg : Config) (es : List Event) (now : Nat) (v : Twist)
    (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords)))
    (hact : phaseAfter .unconfigured es = .active) (hf : v.finite = true)
    (he : cfg.sourceEnabled = true) (ht : 0 < cfg.sourceTimeout)
    (hn : NoScanPoints (run cfg (State.init cfg) es).1 scanPts) :
    (step cfg (run cfg (State.init cfg) es).1 (.cmd now v scanPts zoneVerts)).2.velocity =
      if (run cfg (State.init cfg) es).1.prevZero = true ∧
        (run cfg (State.init cfg) es).1.stopStamp + cfg.stopPubTimeout < now
      then none else some .stopSource :=
  (no_transform cfg _ now v scanPts zoneVerts (by rw [run_phase]; exact hact) hf he ht hn).2

/-- **Bad data (b), over a run.** A command with an infinite or NaN
component leaves no trace: the run with it is the run without it, with one
`silent` more. -/
theorem run_cmd_of_not_finite (cfg : Config) (st : State) (es es' : List Event) (now : Nat)
    (v : Twist) (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords)))
    (h : v.finite = false) :
    run cfg st (es ++ .cmd now v scanPts zoneVerts :: es') =
      ((run cfg st (es ++ es')).1,
        (run cfg st es).2 ++ .silent :: (run cfg (run cfg st es).1 es').2) := by
  rw [run_append, run_cons, step_cmd_of_not_finite _ _ _ _ _ _ h, run_append]

/-- **No amplification, over a run.** Every velocity action published
anywhere in any run, from any state, is `pass`, a stop, or a slowdown whose
ratio pattern is below `F64.one` and is the configured ratio of the zone it
names. -/
theorem run_no_amplification (cfg : Config) (st : State) (es : List Event) (o : Output)
    (a : Action) (h : o ∈ (run cfg st es).2) (ha : o.velocity = some a) :
    a = .pass ∨ (∃ k, a = .stopZone k) ∨ a = .stopSource ∨
      ∃ k r, a = .slowdown k r ∧ r < F64.one ∧
        ∃ z, cfg.zones[k]? = some z ∧ z.action = .slowdown r := by
  obtain ⟨n, hn⟩ := List.mem_iff_getElem?.mp h
  rw [run_getElem?] at hn
  cases he : es[n]? with
  | none => simp [he] at hn
  | some e =>
    rw [he] at hn
    simp only [Option.map_some, Option.some.injEq] at hn
    subst hn
    exact no_amplification cfg _ e a ha

/-- **Slowdown bound, over a run.** After any events since start-up: if the
source is enabled, the scan in force is not too old, the command carries a
coordinates `c` that fit it, and the scan's points under `c` trigger a slowdown
zone with a fixed shape and ratio pattern `rz` below that of 1.0, then the
action demanded for the next command is a stop by a zone or a slowdown with
ratio pattern at most `rz`. -/
theorem run_slowdown_bound (cfg : Config) (es : List Event) (now : Nat) (c : List Coords)
    (zoneVerts : List (Option (List Coords))) (s : Scan) (j : Nat) (z : Zone) (sh : Shape)
    (rz : Nat)
    (he : cfg.sourceEnabled = true) (hlast : lastScan es .unconfigured none = some s)
    (hfresh : ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now))
    (hu : s.fits c = true) (hz : cfg.zones[j]? = some z) (hfix : z.shape = .fixed sh)
    (ha : z.action = .slowdown rz) (ht : z.triggered (s.points c) = true) (hlt : rz < F64.one) :
    (∃ k, evaluate cfg (run cfg (State.init cfg) es).1 now (some c) zoneVerts = .stopZone k) ∨
      ∃ k r, evaluate cfg (run cfg (State.init cfg) es).1 now (some c) zoneVerts = .slowdown k r ∧
        r ≤ rz ∧
        IsBestSlowdown (liveConfig cfg (run cfg (State.init cfg) es).1 zoneVerts).zones
          (s.points c) k r :=
  slowdown_bound cfg _ now (some c) zoneVerts (s.points c) z rz
    ((run_sourcePoints_eq_some_iff cfg es now _ _).mpr
      (Or.inr (Or.inr ⟨he, s, hlast, hfresh, Or.inl ⟨c, rfl, hu, rfl⟩⟩)))
    (List.mem_of_getElem? (liveConfig_of_fixed cfg _ zoneVerts j z sh hz hfix)) ha ht hlt

/-- **Pass-through, over a run.** After any events since start-up that leave
the node active: if the scan in force is not too old, the command carries a
coordinates `c` that fit it, and the scan's points under `c` trigger none of the
zones in force, a finite command that is not zero in the published
components is published unchanged. -/
theorem run_pass_through (cfg : Config) (es : List Event) (now : Nat) (v : Twist) (c : List Coords)
    (zoneVerts : List (Option (List Coords))) (s : Scan)
    (hact : phaseAfter .unconfigured es = .active)
    (hf : v.finite = true) (hnz : v.planarZero = false)
    (he : cfg.sourceEnabled = true) (hlast : lastScan es .unconfigured none = some s)
    (hfresh : ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now))
    (hu : s.fits c = true)
    (hno : ∀ z ∈ (liveConfig cfg (run cfg (State.init cfg) es).1 zoneVerts).zones,
      z.triggered (s.points c) = false) :
    (step cfg (run cfg (State.init cfg) es).1 (.cmd now v (some c) zoneVerts)).2.velocity =
      some .pass :=
  pass_through_of_nonzero cfg _ now v (some c) zoneVerts (s.points c)
    (by rw [run_phase]; exact hact) hf hnz
    ((run_sourcePoints_eq_some_iff cfg es now _ _).mpr
      (Or.inr (Or.inr ⟨he, s, hlast, hfresh, Or.inl ⟨c, rfl, hu, rfl⟩⟩)))
    hno

/-! ## No shape, over a run (N32) -/

/-- An event other than a shape message for zone `k` does not give zone `k`
a shape. -/
theorem step_no_shape (cfg : Config) (st : State) (e : Event) (k : Nat)
    (hn : st.shapes.getD k none = none) (he : ∀ verts foreign, e ≠ .shape k verts foreign) :
    (step cfg st e).1.shapes.getD k none = none := by
  cases e with
  | cmd now v scanPts zoneVerts =>
    by_cases hp : st.phase = .active
    · cases hf : v.finite
      · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; exact hn
      · rw [step_cmd_shapes _ _ _ _ _ _ hp hf]
        unfold shapesAfter
        split
        · exact hn
        · exact (refreshShapes_getD_eq_none_iff cfg st.shapes zoneVerts k).mpr hn
    · rw [step_cmd_of_not_active _ _ _ _ _ _ hp]; exact hn
  | shape j verts foreign =>
    rw [step_shape]
    split
    · have hjk : j ≠ k := by
        rintro rfl
        exact he verts foreign rfl
      simp only [List.getD_eq_getElem?_getD] at hn ⊢
      rw [List.getElem?_set_ne hjk]
      exact hn
    · exact hn
  | cleanup =>
    rw [step_cleanup]
    split
    · simp only [List.getD_eq_getElem?_getD, List.getElem?_map]
      cases st.shapes[k]? <;> rfl
    · exact hn
  | configure => rw [step_configure]; split <;> exact hn
  | activate => rw [step_activate]; split <;> exact hn
  | deactivate => rw [step_deactivate]; split <;> exact hn
  | shutdown => exact hn
  | scan now s => rw [step_scan]; split <;> exact hn

/-- **No shape, over a run.** From a state that holds no shape for zone `k`,
over events none of which is a shape message for zone `k`, the state still
holds none. (An accepted shape message for zone `k` is the only way to give
it one; a cleanup is the only way to take it away.) -/
theorem run_no_shape (cfg : Config) (st : State) (es : List Event) (k : Nat)
    (hn : st.shapes.getD k none = none)
    (he : ∀ e ∈ es, ∀ verts foreign, e ≠ .shape k verts foreign) :
    (run cfg st es).1.shapes.getD k none = none := by
  induction es generalizing st with
  | nil => exact hn
  | cons e es ih =>
    rw [run_cons]
    exact ih _ (step_no_shape cfg st e k hn (he e List.mem_cons_self))
      (fun e' he' => he e' (List.mem_cons_of_mem _ he'))

/-- A reachable unconfigured state holds no shape: at start-up, and after
every cleanup that took effect. -/
theorem no_shape_of_unconfigured (cfg : Config) (st : State) (k : Nat) (hinv : Inv cfg st)
    (hp : st.phase = .unconfigured) : st.shapes.getD k none = none := by
  rw [List.getD_eq_getElem?_getD]
  cases h : st.shapes[k]? with
  | none => rfl
  | some o =>
    have := (hinv.empty hp).2 o (List.mem_of_getElem? h)
    simp [this]

/-- **No shape: no output names the zone, over a run.** Let `pre` be any
events since start-up after which the node is unconfigured (none at all, or
a history that ends with a cleanup that took effect), and `mid` any events
that follow, none of them a shape message for the topic zone `k`. Then the
output for the next event names zone `k` neither as a velocity action nor in
a report.

So before the first accepted shape message for zone `k` after the last
cleanup, zone `k` never acts. This is upstream's behaviour (N32) and it is
**not** fail-safe: a stop zone without shape does not stop. -/
theorem run_output_of_no_shape (cfg : Config) (pre mid : List Event) (e : Event) (k : Nat)
    (z0 : Zone) (h0 : cfg.zones[k]? = some z0) (ht : z0.shape = .topic)
    (hpre : phaseAfter .unconfigured pre = .unconfigured)
    (hmid : ∀ e' ∈ mid, ∀ verts foreign, e' ≠ .shape k verts foreign) :
    let o := (step cfg (run cfg (State.init cfg) (pre ++ mid)).1 e).2
    o.velocity ≠ some (.stopZone k) ∧ (∀ r, o.velocity ≠ some (.slowdown k r)) ∧
      o.report ≠ some (.stop k) ∧ o.report ≠ some (.slowdown k) := by
  intro o
  apply output_of_no_shape cfg _ e k z0 h0 ht
  rw [run_append]
  apply run_no_shape _ _ _ _ _ hmid
  exact no_shape_of_unconfigured cfg _ k (reachable_inv cfg pre) (by rw [run_phase]; exact hpre)

/-! ## After a message with an unusable vertex, over a run (ND6) -/

/-- The mark of an unusable message for zone `k` is neither set nor removed
by events among which there is no polygon message for zone `k` and no
cleanup. -/
theorem run_keeps_unusable (cfg : Config) (st : State) (es : List Event) (k : Nat)
    (he : ∀ e ∈ es, (∀ verts foreign, e ≠ .shape k verts foreign) ∧ e ≠ .cleanup) :
    shapeUnusable (run cfg st es).1.shapes k = shapeUnusable st.shapes k := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih =>
    rw [run_cons, ih _ (fun e' he' => he e' (List.mem_cons_of_mem _ he'))]
    exact step_keeps_unusable cfg st e k (he e List.mem_cons_self).1 (he e List.mem_cons_self).2

/-- **After an unusable message, over a run (review CM5, finding 1).** Let
zone `k` be an enabled stop zone fed by a topic. After any events since
start-up (`pre`), let a polygon message for zone `k` install the mark of an
unusable message (exactly when: `acceptShape_unusable_iff`), and let any
events follow among which there is no polygon message for zone `k` and no
cleanup (`mid`). Then whatever velocity the node publishes for the next
event is a stop: until a later message for the zone is taken or the node is
cleaned up, no command passes and none is merely slowed. (Deactivation and
activation in `mid` change nothing about it.) -/
theorem run_stop_after_unusable (cfg : Config) (pre mid : List Event) (e : Event) (k : Nat)
    (z0 : Zone) (h0 : cfg.zones[k]? = some z0) (ht : z0.shape = .topic) (ha : z0.action = .stop)
    (hen : z0.enabled = true) (verts : Option (List Coords)) (foreign : Bool) (sh : TopicShape)
    (hacc : acceptShape cfg (run cfg (State.init cfg) pre).1 k verts foreign = some sh)
    (hu : sh.unusable = true)
    (hmid : ∀ e' ∈ mid, (∀ verts foreign, e' ≠ .shape k verts foreign) ∧ e' ≠ .cleanup)
    (a : Action)
    (h : (step cfg (run cfg (State.init cfg) (pre ++ .shape k verts foreign :: mid)).1 e).2.velocity
      = some a) : a.isStop = true := by
  apply output_of_unusable cfg _ e k z0 h0 ht ha hen _ a h
  rw [run_append, run_cons, run_keeps_unusable _ _ _ _ hmid]
  have hk : k < (run cfg (State.init cfg) pre).1.shapes.length := by
    rw [(reachable_inv cfg pre).len]
    exact (List.getElem?_eq_some_iff.mp h0).1
  rw [step_shape_unusable _ _ _ _ _ sh hacc hk]
  exact hu

/-! ## After activation, over a run (N6) -/

/-- **After activation, over a run.** After any events since start-up that
leave the node inactive, an `activate` makes it active, and a finite command
that comes next is answered with a velocity, also when that velocity is
zero and `stop_pub_timeout` had run out before the node was deactivated: a
stop is never withheld right after an activation. Its report is published
unless it is `normal`. -/
theorem run_after_activation (cfg : Config) (es : List Event) (now : Nat) (v : Twist)
    (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords)))
    (hin : phaseAfter .unconfigured es = .inactive) (hf : v.finite = true) :
    let st := (run cfg (State.init cfg) es).1
    let st' := (run cfg (State.init cfg) (es ++ [.activate])).1
    st'.phase = .active ∧
      (step cfg st' (.cmd now v scanPts zoneVerts)).2.velocity =
        some (evaluate cfg st now scanPts zoneVerts) ∧
      (step cfg st' (.cmd now v scanPts zoneVerts)).2.report =
        if (evaluate cfg st now scanPts zoneVerts).report v = .normal then none
        else some ((evaluate cfg st now scanPts zoneVerts).report v) := by
  intro st st'
  have hst' : st' = (step cfg st .activate).1 := by
    simp only [st', st, run_concat]
  rw [hst']
  exact after_activation_of_inv cfg st now v scanPts zoneVerts (reachable_inv cfg es)
    (by simp only [st]; rw [run_phase]; exact hin) hf

/-! ## `stop_pub_timeout`: what the state records, and what silence means -/

/-- R15: a processed command for which zero is demanded, after a command for
which it was not, starts a period of zero: it is published, and the stop
stamp becomes its time. -/
theorem step_zero_start (cfg : Config) (st : State) (t : Nat) (v : Twist)
    (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords))) (hact : st.phase = .active) (hf : v.finite = true)
    (hp : st.prevZero = false) (hz : (evaluate cfg st t scanPts zoneVerts).yieldsZero v = true) :
    (step cfg st (.cmd t v scanPts zoneVerts)).2.velocity =
        some (evaluate cfg st t scanPts zoneVerts) ∧
      (step cfg st (.cmd t v scanPts zoneVerts)).1.prevZero = true ∧
      (step cfg st (.cmd t v scanPts zoneVerts)).1.stopStamp = t ∧
      (step cfg st (.cmd t v scanPts zoneVerts)).1.phase = .active := by
  refine ⟨velocity_of_not_prevZero cfg st t v scanPts zoneVerts hact hf hp, ?_, ?_, ?_⟩
  · rw [step_cmd_prevZero _ _ _ _ _ _ hact hf, hz]
  · rw [step_cmd_stopStamp _ _ _ _ _ _ hact hf]
    simp [hz, hp]
  · rw [(step_cmd_scan _ _ _ _ _ _).2, hact]

/-- An event that is neither a command nor a `deactivate` leaves the record
of zeros (R15) alone. -/
theorem step_zero_record (cfg : Config) (st : State) (e : Event)
    (hc : ∀ now v scanPts zoneVerts, e ≠ .cmd now v scanPts zoneVerts) (hd : e ≠ .deactivate) :
    (step cfg st e).1.prevZero = st.prevZero ∧ (step cfg st e).1.stopStamp = st.stopStamp := by
  cases e with
  | cmd now v scanPts zoneVerts => exact absurd rfl (hc now v scanPts zoneVerts)
  | deactivate => exact absurd rfl hd
  | shape k verts foreign => rw [step_shape]; split <;> exact ⟨rfl, rfl⟩
  | shutdown => exact ⟨rfl, rfl⟩
  | configure => rw [step_configure]; split <;> exact ⟨rfl, rfl⟩
  | activate => rw [step_activate]; split <;> exact ⟨rfl, rfl⟩
  | cleanup => rw [step_cleanup]; split <;> exact ⟨rfl, rfl⟩
  | scan now s => rw [step_scan]; split <;> exact ⟨rfl, rfl⟩

/-- R15: during a period of zero the record does not move. If the previous
demanded velocity was zero, no `deactivate` occurs in `mid`, and every
velocity action published during `mid` yields zero for its command, then
afterwards the previous demanded velocity is still zero and the stop stamp
is unchanged. Scans, shape messages, other lifecycle events, ignored
commands and commands whose velocity is withheld may occur in `mid`. -/
theorem run_zero_streak (cfg : Config) (st : State) (mid : List Event)
    (hp : st.prevZero = true) (hd : ∀ e ∈ mid, e ≠ .deactivate)
    (hz : ∀ (j now : Nat) (v : Twist) (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords)))
      (o : Output) (a : Action), mid[j]? = some (.cmd now v scanPts zoneVerts) →
      (run cfg st mid).2[j]? = some o → o.velocity = some a → a.yieldsZero v = true) :
    (run cfg st mid).1.prevZero = true ∧ (run cfg st mid).1.stopStamp = st.stopStamp := by
  induction mid generalizing st with
  | nil => exact ⟨hp, rfl⟩
  | cons e es ih =>
    have hstep : (step cfg st e).1.prevZero = true ∧
        (step cfg st e).1.stopStamp = st.stopStamp := by
      by_cases hc : ∃ now v scanPts zoneVerts, e = .cmd now v scanPts zoneVerts
      · obtain ⟨now, v, scanPts, zoneVerts, rfl⟩ := hc
        by_cases hact : st.phase = .active
        · cases hf : v.finite
          · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; exact ⟨hp, rfl⟩
          · have hyz : (evaluate cfg st now scanPts zoneVerts).yieldsZero v = true := by
              rcases velocity_eq_none_or cfg st now v scanPts zoneVerts with h | h
              · rw [velocity_eq_none_iff] at h
                rcases h with h | h | h
                · exact absurd hact h
                · rw [hf] at h; cases h
                · exact h.1
              · exact hz 0 now v scanPts zoneVerts _ _ rfl (by rw [run_cons]; rfl) h
            rw [step_cmd_prevZero _ _ _ _ _ _ hact hf, step_cmd_stopStamp _ _ _ _ _ _ hact hf]
            simp [hyz, hp]
        · rw [step_cmd_of_not_active _ _ _ _ _ _ hact]; exact ⟨hp, rfl⟩
      · have := step_zero_record cfg st e (fun now v s z he => hc ⟨now, v, s, z, he⟩)
          (hd e List.mem_cons_self)
        exact ⟨by rw [this.1]; exact hp, this.2⟩
    have := ih (step cfg st e).1 hstep.1 (fun e' he' => hd e' (List.mem_cons_of_mem _ he'))
      (fun j now v scanPts zoneVerts o a hj ho =>
        hz (j + 1) now v scanPts zoneVerts o a (by simpa using hj)
          (by rw [run_cons]; simpa using ho))
    rw [run_cons]
    exact ⟨this.1, this.2.trans hstep.2⟩

/-- **A period of zero (R15).** Suppose a processed command at time `t` is
the first for which zero is demanded (the previous demanded velocity was not
zero), no `deactivate` occurs in the events `mid` that follow, and every
velocity action published during `mid` yields zero for its command. Then a
finite command at `now'` after `mid`, with the node still active, gets the
demanded action as its velocity, except when that action yields zero and
`t + stop_pub_timeout < now'`: then the velocity is withheld.

So zeros are published from `t` up to and including `t + stop_pub_timeout`,
whatever their cause (a stop zone, an invalid source, a zero command, a zero
ratio), and a non-zero demand is always published. -/
theorem zero_period (cfg : Config) (st : State) (t : Nat) (v : Twist)
    (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords))) (mid : List Event) (now' : Nat) (v' : Twist)
    (scanPts' : Option (List Coords)) (zoneVerts' : List (Option (List Coords)))
    (hact : st.phase = .active) (hf : v.finite = true) (hp : st.prevZero = false)
    (hz : (evaluate cfg st t scanPts zoneVerts).yieldsZero v = true)
    (hd : ∀ e ∈ mid, e ≠ .deactivate)
    (hmid : ∀ (j now₁ : Nat) (v₁ : Twist) (s₁ : Option (List Coords))
      (z₁ : List (Option (List Coords)))
      (o : Output) (a : Action), mid[j]? = some (.cmd now₁ v₁ s₁ z₁) →
      (run cfg (step cfg st (.cmd t v scanPts zoneVerts)).1 mid).2[j]? = some o →
        o.velocity = some a → a.yieldsZero v₁ = true)
    (hact' : (run cfg (step cfg st (.cmd t v scanPts zoneVerts)).1 mid).1.phase = .active)
    (hf' : v'.finite = true) :
    (step cfg (run cfg (step cfg st (.cmd t v scanPts zoneVerts)).1 mid).1
        (.cmd now' v' scanPts' zoneVerts')).2.velocity =
      if (evaluate cfg (run cfg (step cfg st (.cmd t v scanPts zoneVerts)).1 mid).1 now' scanPts'
          zoneVerts').yieldsZero v' = true ∧ t + cfg.stopPubTimeout < now'
      then none
      else some (evaluate cfg (run cfg (step cfg st (.cmd t v scanPts zoneVerts)).1 mid).1 now'
        scanPts' zoneVerts') := by
  obtain ⟨-, hp1, hs1, -⟩ := step_zero_start cfg st t v scanPts zoneVerts hact hf hp hz
  obtain ⟨hp2, hs2⟩ := run_zero_streak cfg _ mid hp1 hd hmid
  rw [hs1] at hs2
  rw [velocity_cmd _ _ _ _ _ _ hact' hf', hp2, hs2]
  simp

/-- The velocity published last before position `n` of the run from start-up
over `es` was a zero: some earlier command got a velocity action that yields
zero for it, and no velocity was published between it and position `n`. -/
def ZeroPublishedLast (cfg : Config) (es : List Event) (n : Nat) : Prop :=
  ∃ m now v scanPts zoneVerts o a, m < n ∧ es[m]? = some (.cmd now v scanPts zoneVerts) ∧
    (run cfg (State.init cfg) es).2[m]? = some o ∧ o.velocity = some a ∧
    a.yieldsZero v = true ∧
    ∀ j, m < j → j < n → ∀ o', (run cfg (State.init cfg) es).2[j]? = some o' → o'.velocity = none

private theorem ZeroPublishedLast.succ {cfg : Config} {es : List Event} {n : Nat}
    (h : ZeroPublishedLast cfg es n)
    (hn : ∀ o', (run cfg (State.init cfg) es).2[n]? = some o' → o'.velocity = none) :
    ZeroPublishedLast cfg es (n + 1) := by
  obtain ⟨m, now, v, scanPts, zoneVerts, o, a, hm, he, ho, hv, hz, hbetween⟩ := h
  refine ⟨m, now, v, scanPts, zoneVerts, o, a, by omega, he, ho, hv, hz, ?_⟩
  intro j hj hj' o' ho'
  by_cases hjn : j = n
  · subst hjn; exact hn o' ho'
  · exact hbetween j hj (by omega) o' ho'

/-- What `prevZero` means in a run from start-up: if it is set after the
first `n` events, the velocity published last was a zero. -/
theorem prevZero_zeroPublishedLast (cfg : Config) (es : List Event) (n : Nat)
    (h : (run cfg (State.init cfg) (es.take n)).1.prevZero = true) :
    ZeroPublishedLast cfg es n := by
  induction n with
  | zero => simp [run_nil, State.init] at h
  | succ n ih =>
    cases he : es[n]? with
    | none =>
      rw [List.take_add_one, he, Option.toList_none, List.append_nil] at h
      refine (ih h).succ ?_
      intro o' ho'
      rw [run_getElem?, he] at ho'
      cases ho'
    | some e =>
      have hout := run_output cfg (State.init cfg) es n e he
      rw [run_take_succ cfg _ es n e he] at h
      by_cases hc : ∃ now v scanPts zoneVerts, e = .cmd now v scanPts zoneVerts
      · obtain ⟨now, v, scanPts, zoneVerts, rfl⟩ := hc
        by_cases hact : (run cfg (State.init cfg) (es.take n)).1.phase = .active
        · cases hf : v.finite
          · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf] at h hout
            refine (ih h).succ ?_
            intro o' ho'
            rw [hout] at ho'
            cases ho'
            rfl
          · rw [step_cmd_prevZero _ _ _ _ _ _ hact hf] at h
            rcases velocity_eq_none_or cfg (run cfg (State.init cfg) (es.take n)).1 now v scanPts
              zoneVerts with hs | hp
            · have hprev := (velocity_eq_none_iff _ _ _ _ _ _).mp hs
              rcases hprev with hprev | hprev | hprev
              · exact absurd hact hprev
              · rw [hf] at hprev; cases hprev
              · refine (ih hprev.2.1).succ ?_
                intro o' ho'
                rw [hout] at ho'
                cases ho'
                exact hs
            · exact ⟨n, now, v, scanPts, zoneVerts, _, _, Nat.lt_succ_self n, he, hout, hp, h,
                fun j hj hj' => by omega⟩
        · rw [step_cmd_of_not_active _ _ _ _ _ _ hact] at h hout
          refine (ih h).succ ?_
          intro o' ho'
          rw [hout] at ho'
          cases ho'
          rfl
      · have hsil : (step cfg (run cfg (State.init cfg) (es.take n)).1 e).2 = .silent :=
          output_of_not_cmd cfg _ e (fun now v s z he' => hc ⟨now, v, s, z, he'⟩)
        have hprev : (run cfg (State.init cfg) (es.take n)).1.prevZero = true := by
          by_cases hd : e = .deactivate
          · subst hd
            rw [step_deactivate] at h
            split at h
            · simp at h
            · exact h
          · rw [(step_zero_record cfg _ e (fun now v s z he' => hc ⟨now, v, s, z, he'⟩) hd).1] at h
            exact h
        refine (ih hprev).succ ?_
        intro o' ho'
        rw [hout, hsil] at ho'
        cases ho'
        rfl

/-- **A withheld velocity follows a published zero.** In any run from
start-up, over events of all kinds, if a command is processed and gets no
velocity (the output is `answer none _`), then the velocity published last
before it was a zero: an earlier command was answered with an action that
yields zero for it (a stop, a zero command passed on, or a zero ratio), and
no velocity has been published since, nor has the node been deactivated
since.

This is what the robot's base has received when the node withholds under
R15, also when the cause of the zero changes in between: after a zero
command has been repeated for longer than `stop_pub_timeout`, an obstacle
that enters a stop zone together with a non-zero command gets no velocity
message, not a stop message; the last message the base received is the
earlier zero. A base that holds its last command stays stopped; a base that
needs a fresh zero to stay stopped needs its own watchdog (SPEC, A3). -/
theorem run_withheld_last_published_zero (cfg : Config) (es : List Event) (n : Nat)
    (rep : Option Report) (hs : (run cfg (State.init cfg) es).2[n]? = some (.answer none rep)) :
    ZeroPublishedLast cfg es n := by
  obtain ⟨now, v, scanPts, zoneVerts, he, hact, hf⟩ :=
    run_processed cfg _ es n _ hs (by simp)
  rw [run_output _ _ _ _ _ he] at hs
  simp only [Option.some.injEq] at hs
  have hv : (step cfg (run cfg (State.init cfg) (es.take n)).1
      (.cmd now v scanPts zoneVerts)).2.velocity = none := by rw [hs]; rfl
  rcases (velocity_eq_none_iff _ _ _ _ _ _).mp hv with h | h | h
  · exact absurd hact h
  · rw [hf] at h; cases h
  · exact prevZero_zeroPublishedLast cfg es n h.2.1

/-- No velocity is withheld before one has been published: the first
processed command after start-up always gets a velocity. -/
theorem run_withheld_after_published (cfg : Config) (es : List Event) (n : Nat)
    (rep : Option Report) (hs : (run cfg (State.init cfg) es).2[n]? = some (.answer none rep)) :
    ∃ m o a, m < n ∧ (run cfg (State.init cfg) es).2[m]? = some o ∧ o.velocity = some a := by
  obtain ⟨m, -, -, -, -, o, a, hm, -, ho, hv, -⟩ :=
    run_withheld_last_published_zero cfg es n rep hs
  exact ⟨m, o, a, hm, ho, hv⟩

end ROSlean.CollisionMonitor
