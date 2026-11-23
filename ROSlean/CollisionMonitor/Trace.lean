import ROSlean.CollisionMonitor.Properties

/-!
# The collision monitor over event sequences

Lifts the single-step theorems of `ROSlean.CollisionMonitor.Properties` to
`run`.

* `run_getElem?` is the trace invariant: the `n`-th output of any run is the
  single-step output for the `n`-th event in the state reached after the
  first `n` events. Every single-step theorem therefore holds at every point
  of every run.
* `run_scan` ties the state to the history: the stored scan is the one
  received last (R5). With it the properties of the SPEC are restated for a
  command that arrives after an arbitrary sequence of events since start-up,
  with hypotheses about the events only (`run_stop`, `run_stale`,
  `run_pass_through`, `run_slowdown_bound`, `run_no_amplification`).
* `stop_pub_timeout` (R15) is the one place where the node is silent while a
  stop is demanded. `run_silent_last_published_zero` says what makes that
  acceptable: whenever a finite command gets no answer, the message
  published last before it was a zero. `run_zero_streak` and `zero_period`
  say what `prevZero` and `stopStamp` are during a period of zero and exactly
  when its zeros are published.

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

/-- The state after the first `n + 1` events is one step from the state after
the first `n`. -/
theorem run_take_succ (cfg : Config) (st : State) (es : List Event) (n : Nat) (e : Event)
    (h : es[n]? = some e) :
    (run cfg st (es.take (n + 1))).1 = (step cfg (run cfg st (es.take n)).1 e).1 := by
  rw [List.take_add_one, h, Option.toList_some, run_concat]

/-- The whole run is total and the function of the events it is defined to
be: nothing but the events enters. Scan events never produce output (R1). -/
theorem run_scan_output (cfg : Config) (st : State) (es : List Event) (n now : Nat) (s : Scan)
    (h : es[n]? = some (.scan now s)) : (run cfg st es).2[n]? = some .silent := by
  rw [run_getElem?, h]
  rfl

/-! ## The stored scan is the one received last (R5) -/

/-- The scan of the last scan event of the sequence, or `d` if it has none.
The last *received*, whatever the stamps say. -/
def lastScan : List Event → Option Scan → Option Scan
  | [], d => d
  | .scan _ s :: es, _ => lastScan es (some s)
  | .cmd _ _ :: es, d => lastScan es d

/-- After any run, the stored scan is the one received last. Commands,
finite or not, do not touch it. -/
theorem run_scan (cfg : Config) (st : State) (es : List Event) :
    (run cfg st es).1.scan = lastScan es st.scan := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih =>
    rw [run_cons, ih]
    cases e with
    | scan now s => rfl
    | cmd now v => rw [step_cmd_scan]; rfl

theorem lastScan_append_scan (es : List Event) (now : Nat) (s : Scan) (d : Option Scan) :
    lastScan (es ++ [.scan now s]) d = some s := by
  induction es generalizing d with
  | nil => rfl
  | cons e es ih => cases e <;> simp [lastScan, ih]

theorem lastScan_append_cmd (es : List Event) (now : Nat) (v : Twist) (d : Option Scan) :
    lastScan (es ++ [.cmd now v]) d = lastScan es d := by
  induction es generalizing d with
  | nil => rfl
  | cons e es ih => cases e <;> simp [lastScan, ih]

/-- The action demanded after any events since start-up depends on the
history only through the scan received last. -/
theorem run_evaluate (cfg : Config) (es : List Event) (now : Nat) :
    evaluate cfg (run cfg State.init es).1 now =
      evaluate cfg { State.init with scan := lastScan es none } now :=
  evaluate_congr _ _ _ _ (run_scan cfg State.init es)

/-- The points tested after any events since start-up, from the history
alone (R5 to R8): `pts` iff the source is disabled and `pts = []`; or the
timeout is 0, no scan has arrived and `pts = []`; or the scan received last
is not too old at `now` and `pts` are its valid points. -/
theorem run_sourcePoints_eq_some_iff (cfg : Config) (es : List Event) (now : Nat)
    (pts : List Point) :
    sourcePoints cfg (run cfg State.init es).1 now = some pts ↔
      (cfg.sourceEnabled = false ∧ pts = []) ∨
      (cfg.sourceEnabled = true ∧ lastScan es none = none ∧ cfg.sourceTimeout = 0 ∧ pts = []) ∨
      (cfg.sourceEnabled = true ∧ ∃ s, lastScan es none = some s ∧
        ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now) ∧ pts = s.points) := by
  rw [sourcePoints_eq_some_iff, run_scan]
  rfl

/-! ## The properties of the SPEC, after any events since start-up

In each statement `es` is an arbitrary sequence of events, and the command
`(now, v)` arrives next. By `run_concat` its output is the last element of
`(run cfg State.init (es ++ [.cmd now v])).2`. -/

/-- **Stop, over a run.** After any events since start-up: if the source is
enabled, the scan received last is not too old and has at least `min_points`
valid points in an enabled stop zone, then the output of the next command is
`publish (stopZone k)` for the first such zone, unless the command is not
finite or zero has already been demanded for more than `stop_pub_timeout`
(see `run_silent_last_published_zero` for what was published then). -/
theorem run_stop (cfg : Config) (es : List Event) (now : Nat) (v : Twist) (s : Scan) (z : Zone)
    (he : cfg.sourceEnabled = true) (hlast : lastScan es none = some s)
    (hfresh : ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now))
    (hz : z ∈ cfg.zones) (hs : z.action = .stop) (ht : z.triggered s.points = true) :
    ∃ k, IsFirstStop cfg.zones s.points k ∧
      (step cfg (run cfg State.init es).1 (.cmd now v)).2 =
        if v.finite = true ∧ ¬ ((run cfg State.init es).1.prevZero = true ∧
          (run cfg State.init es).1.stopStamp + cfg.stopPubTimeout < now)
        then .publish (.stopZone k) else .silent := by
  have hpts : sourcePoints cfg (run cfg State.init es).1 now = some s.points :=
    (run_sourcePoints_eq_some_iff cfg es now _).mpr (Or.inr (Or.inr ⟨he, s, hlast, hfresh, rfl⟩))
  obtain ⟨k, hk, -, hout⟩ := stop cfg _ now v s.points z hpts hz hs ht
  exact ⟨k, hk, hout⟩

/-- **Stale data, over a run.** After any events since start-up: if the
source is enabled with a non-zero timeout, and no scan has arrived or the
stamp of the scan received last is more than the timeout before `now`, the
output of the next command is `publish stopSource`, unless the command is
not finite or zero has already been demanded for more than
`stop_pub_timeout`. -/
theorem run_stale (cfg : Config) (es : List Event) (now : Nat) (v : Twist)
    (he : cfg.sourceEnabled = true) (ht : 0 < cfg.sourceTimeout)
    (hs : lastScan es none = none ∨
      ∃ s, lastScan es none = some s ∧ s.stamp + cfg.sourceTimeout < now) :
    (step cfg (run cfg State.init es).1 (.cmd now v)).2 =
      if v.finite = true ∧ ¬ ((run cfg State.init es).1.prevZero = true ∧
        (run cfg State.init es).1.stopStamp + cfg.stopPubTimeout < now)
      then .publish .stopSource else .silent := by
  refine (stale cfg _ now v he ht ?_).2
  rw [run_scan]
  exact hs

/-- **Bad data (b), over a run.** A command with an infinite or NaN
component leaves no trace: the run with it is the run without it, with one
`silent` more. -/
theorem run_cmd_of_not_finite (cfg : Config) (st : State) (es es' : List Event) (now : Nat)
    (v : Twist) (h : v.finite = false) :
    run cfg st (es ++ .cmd now v :: es') =
      ((run cfg st (es ++ es')).1,
        (run cfg st es).2 ++ .silent :: (run cfg (run cfg st es).1 es').2) := by
  rw [run_append, run_cons, step_cmd_of_not_finite _ _ _ _ h, run_append]

/-- **No amplification, over a run.** Every action published anywhere in any
run, from any state, is `pass`, a stop, or a slowdown whose ratio pattern is
below `F64.one` and is the configured ratio of the zone it names. -/
theorem run_no_amplification (cfg : Config) (st : State) (es : List Event) (a : Action)
    (h : .publish a ∈ (run cfg st es).2) :
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
    cases e with
    | scan now s => simp [step_scan] at hn
    | cmd now v => exact no_amplification cfg _ now v a hn

/-- **Slowdown bound, over a run.** After any events since start-up: if the
source is enabled, the scan received last is not too old, and its valid
points trigger a slowdown zone with ratio pattern `rz` below that of 1.0,
then the action demanded for the next command is a stop by a zone or a
slowdown with ratio pattern at most `rz`. -/
theorem run_slowdown_bound (cfg : Config) (es : List Event) (now : Nat) (s : Scan) (z : Zone)
    (rz : Nat) (he : cfg.sourceEnabled = true) (hlast : lastScan es none = some s)
    (hfresh : ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now))
    (hz : z ∈ cfg.zones) (ha : z.action = .slowdown rz) (ht : z.triggered s.points = true)
    (hlt : rz < F64.one) :
    (∃ k, evaluate cfg (run cfg State.init es).1 now = .stopZone k) ∨
      ∃ k r, evaluate cfg (run cfg State.init es).1 now = .slowdown k r ∧ r ≤ rz ∧
        IsBestSlowdown cfg.zones s.points k r :=
  slowdown_bound cfg _ now s.points z rz
    ((run_sourcePoints_eq_some_iff cfg es now _).mpr (Or.inr (Or.inr ⟨he, s, hlast, hfresh, rfl⟩)))
    hz ha ht hlt

/-- **Pass-through, over a run.** After any events since start-up: if the
scan received last is not too old and its valid points trigger no zone, a
finite command that is not zero in the published components is published
unchanged. -/
theorem run_pass_through (cfg : Config) (es : List Event) (now : Nat) (v : Twist) (s : Scan)
    (hf : v.finite = true) (hnz : v.planarZero = false)
    (he : cfg.sourceEnabled = true) (hlast : lastScan es none = some s)
    (hfresh : ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now))
    (hno : ∀ z ∈ cfg.zones, z.triggered s.points = false) :
    (step cfg (run cfg State.init es).1 (.cmd now v)).2 = .publish .pass :=
  pass_through_of_nonzero cfg _ now v s.points hf hnz
    ((run_sourcePoints_eq_some_iff cfg es now _).mpr (Or.inr (Or.inr ⟨he, s, hlast, hfresh, rfl⟩)))
    hno

/-! ## `stop_pub_timeout`: what the state records, and what silence means -/

/-- R15: a finite command for which zero is demanded, after a command for
which it was not, starts a period of zero: it is published, and the stop
stamp becomes its time. -/
theorem step_zero_start (cfg : Config) (st : State) (t : Nat) (v : Twist) (hf : v.finite = true)
    (hp : st.prevZero = false) (hz : (evaluate cfg st t).yieldsZero v = true) :
    (step cfg st (.cmd t v)).2 = .publish (evaluate cfg st t) ∧
      (step cfg st (.cmd t v)).1.prevZero = true ∧ (step cfg st (.cmd t v)).1.stopStamp = t := by
  refine ⟨output_of_not_prevZero cfg st t v hf hp, ?_, ?_⟩
  · rw [step_cmd_prevZero _ _ _ _ hf, hz]
  · rw [step_cmd_stopStamp _ _ _ _ hf]
    simp [hz, hp]

/-- R15: during a period of zero the state does not move. If the previous
demanded velocity was zero, and every action published during `mid` yields
zero for its command, then afterwards the previous demanded velocity is
still zero and the stop stamp is unchanged. Scans, ignored commands and
commands that get no answer may occur in `mid`. -/
theorem run_zero_streak (cfg : Config) (st : State) (mid : List Event)
    (hp : st.prevZero = true)
    (hz : ∀ (j now : Nat) (v : Twist) (a : Action), mid[j]? = some (.cmd now v) →
      (run cfg st mid).2[j]? = some (.publish a) → a.yieldsZero v = true) :
    (run cfg st mid).1.prevZero = true ∧ (run cfg st mid).1.stopStamp = st.stopStamp := by
  induction mid generalizing st with
  | nil => exact ⟨hp, rfl⟩
  | cons e es ih =>
    have hstep : (step cfg st e).1.prevZero = true ∧
        (step cfg st e).1.stopStamp = st.stopStamp := by
      cases e with
      | scan now s => exact ⟨hp, rfl⟩
      | cmd now v =>
        by_cases hf : v.finite = true
        · have hyz : (evaluate cfg st now).yieldsZero v = true := by
            rcases output_eq_silent_or_publish cfg st now v with h | h
            · rw [output_eq_silent_iff] at h
              rcases h with h | h
              · rw [hf] at h
                exact absurd h (by simp)
              · exact h.1
            · exact hz 0 now v _ rfl (by rw [run_cons]; simpa using h)
          rw [step_cmd_prevZero _ _ _ _ hf, step_cmd_stopStamp _ _ _ _ hf]
          simp [hyz, hp]
        · rw [step_cmd_of_not_finite _ _ _ _ (by simpa using hf)]
          exact ⟨hp, rfl⟩
    have := ih (step cfg st e).1 hstep.1 (fun j now v a hj ho =>
      hz (j + 1) now v a (by simpa using hj) (by rw [run_cons]; simpa using ho))
    rw [run_cons]
    exact ⟨this.1, this.2.trans hstep.2⟩

/-- **A period of zero (R15).** Suppose a finite command at time `t` is the
first for which zero is demanded (the previous demanded velocity was not
zero), and every action published during the events `mid` that follow
yields zero for its command. Then a finite command at `now'` after `mid` is
answered with the demanded action, except when that action yields zero and
`t + stop_pub_timeout < now'`: then nothing is published.

So zeros are published from `t` up to and including `t + stop_pub_timeout`,
whatever their cause (a stop zone, an invalid source, a zero command, a zero
ratio), and a non-zero demand is always published. -/
theorem zero_period (cfg : Config) (st : State) (t : Nat) (v : Twist) (mid : List Event)
    (now' : Nat) (v' : Twist) (hf : v.finite = true) (hp : st.prevZero = false)
    (hz : (evaluate cfg st t).yieldsZero v = true)
    (hmid : ∀ (j now₁ : Nat) (v₁ : Twist) (a : Action), mid[j]? = some (.cmd now₁ v₁) →
      (run cfg (step cfg st (.cmd t v)).1 mid).2[j]? = some (.publish a) →
        a.yieldsZero v₁ = true)
    (hf' : v'.finite = true) :
    (step cfg (run cfg (step cfg st (.cmd t v)).1 mid).1 (.cmd now' v')).2 =
      if (evaluate cfg (run cfg (step cfg st (.cmd t v)).1 mid).1 now').yieldsZero v' = true ∧
        t + cfg.stopPubTimeout < now'
      then .silent
      else .publish (evaluate cfg (run cfg (step cfg st (.cmd t v)).1 mid).1 now') := by
  obtain ⟨-, hp1, hs1⟩ := step_zero_start cfg st t v hf hp hz
  obtain ⟨hp2, hs2⟩ := run_zero_streak cfg _ mid hp1 hmid
  rw [hs1] at hs2
  split
  · rename_i h
    rw [output_eq_silent_iff]
    exact Or.inr ⟨h.1, hp2, by rw [hs2]; exact h.2⟩
  · rename_i h
    rw [output_eq_publish_iff]
    refine ⟨hf', rfl, fun hc => h ⟨hc.1, ?_⟩⟩
    rw [← hs2]
    exact hc.2.2

/-- The message published last before position `n` of the run from start-up
over `es` was a zero: some earlier command got an action that yields zero
for it, and nothing was published between it and position `n`. -/
def ZeroPublishedLast (cfg : Config) (es : List Event) (n : Nat) : Prop :=
  ∃ m now v a, m < n ∧ es[m]? = some (.cmd now v) ∧
    (run cfg State.init es).2[m]? = some (.publish a) ∧ a.yieldsZero v = true ∧
    ∀ j, m < j → j < n → ∀ b, (run cfg State.init es).2[j]? ≠ some (.publish b)

private theorem ZeroPublishedLast.succ {cfg : Config} {es : List Event} {n : Nat}
    (h : ZeroPublishedLast cfg es n)
    (hn : ∀ b, (run cfg State.init es).2[n]? ≠ some (.publish b)) :
    ZeroPublishedLast cfg es (n + 1) := by
  obtain ⟨m, now, v, a, hm, he, ho, hz, hbetween⟩ := h
  refine ⟨m, now, v, a, by omega, he, ho, hz, ?_⟩
  intro j hj hj' b
  by_cases hjn : j = n
  · rw [hjn]; exact hn b
  · exact hbetween j hj (by omega) b

/-- What `prevZero` means in a run from start-up: if it is set after the
first `n` events, the message published last was a zero. -/
theorem prevZero_zeroPublishedLast (cfg : Config) (es : List Event) (n : Nat)
    (h : (run cfg State.init (es.take n)).1.prevZero = true) : ZeroPublishedLast cfg es n := by
  induction n with
  | zero => simp [run_nil, State.init] at h
  | succ n ih =>
    cases he : es[n]? with
    | none =>
      rw [List.take_add_one, he, Option.toList_none, List.append_nil] at h
      refine (ih h).succ ?_
      intro b
      rw [run_getElem?, he]
      simp
    | some e =>
      have hout : (run cfg State.init es).2[n]? =
          some (step cfg (run cfg State.init (es.take n)).1 e).2 := by
        rw [run_getElem?, he]; rfl
      rw [run_take_succ cfg _ es n e he] at h
      cases e with
      | scan now s =>
        refine (ih h).succ ?_
        intro b
        rw [hout]
        simp [step_scan]
      | cmd now v =>
        by_cases hf : v.finite = true
        · rw [step_cmd_prevZero _ _ _ _ hf] at h
          rcases output_eq_silent_or_publish cfg (run cfg State.init (es.take n)).1 now v
            with hs | hp
          · have hprev := ((output_eq_silent_iff _ _ _ _).mp hs).resolve_left (by simp [hf])
            refine (ih hprev.2.1).succ ?_
            intro b
            rw [hout, hs]
            simp
          · exact ⟨n, now, v, _, Nat.lt_succ_self n, he, by rw [hout, hp], h,
              fun j hj hj' => by omega⟩
        · have hnf : v.finite = false := by simpa using hf
          rw [step_cmd_of_not_finite _ _ _ _ hnf] at h
          refine (ih h).succ ?_
          intro b
          rw [hout, step_cmd_of_not_finite _ _ _ _ hnf]
          simp

/-- **Silence follows a published zero.** In any run from start-up, if a
finite command gets no answer, then the message published last before it
was a zero: an earlier command was answered with an action that yields zero
for it (a stop, a zero command passed on, or a zero ratio), and nothing has
been published since.

This is what the robot's base has received when the node goes silent under
R15, also when the cause of the zero changes in between: after a zero
command has been repeated for longer than `stop_pub_timeout`, an obstacle
that enters a stop zone together with a non-zero command is answered with
silence, not with a stop message; the last message the base received is the
earlier zero. A base that holds its last command stays stopped; a base that
needs a fresh zero to stay stopped needs its own watchdog (SPEC, A3). -/
theorem run_silent_last_published_zero (cfg : Config) (es : List Event) (n now : Nat)
    (v : Twist) (he : es[n]? = some (.cmd now v)) (hf : v.finite = true)
    (hs : (run cfg State.init es).2[n]? = some .silent) : ZeroPublishedLast cfg es n := by
  rw [run_getElem?, he] at hs
  simp only [Option.map_some, Option.some.injEq] at hs
  have hprev := ((output_eq_silent_iff _ _ _ _).mp hs).resolve_left (by simp [hf])
  exact prevZero_zeroPublishedLast cfg es n hprev.2.1

/-- The first finite command after start-up is always answered: there is no
earlier zero that could excuse silence. -/
theorem run_first_command_answered (cfg : Config) (es : List Event) (n now : Nat) (v : Twist)
    (he : es[n]? = some (.cmd now v)) (hf : v.finite = true)
    (hfirst : ∀ m now' v', m < n → es[m]? = some (.cmd now' v') → v'.finite = false) :
    ∃ a, (run cfg State.init es).2[n]? = some (.publish a) := by
  have hout : (run cfg State.init es).2[n]? =
      some (step cfg (run cfg State.init (es.take n)).1 (.cmd now v)).2 := by
    rw [run_getElem?, he]; rfl
  rcases output_eq_silent_or_publish cfg (run cfg State.init (es.take n)).1 now v with h | h
  · exfalso
    obtain ⟨m, now', v', a, hm, hem, hom, -, -⟩ :=
      run_silent_last_published_zero cfg es n now v he hf (by rw [hout, h])
    have hnf := hfirst m now' v' hm hem
    rw [run_getElem?, hem] at hom
    simp [step_cmd_of_not_finite _ _ _ _ hnf] at hom
  · exact ⟨_, by rw [hout, h]⟩

end ROSlean.CollisionMonitor
