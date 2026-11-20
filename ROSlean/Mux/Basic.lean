import Mathlib.Tactic
import ROSleanCore.Mux

/-!
# Basic facts about the twist_mux model

Characterisations of the building blocks of `ROSleanCore.Mux`: the padded
write, the lock-priority fold and the selection loop. The theorems that
correspond to the property table are in `ROSlean.Mux.Properties`.
-/

namespace ROSlean.Mux

open ROSleanCore.Mux

/-! ## State access -/

/-- `setPad` and `getD` behave as a total map: reading back the written index
gives the written value, and every other index is unchanged. -/
theorem getD_setPad {α : Type} (l : List α) (i k : Nat) (v d : α) :
    (setPad l i v d).getD k d = if k = i then v else l.getD k d := by
  unfold setPad
  by_cases hi : i < l.length
  · simp only [hi, ite_true, List.getD_eq_getElem?_getD, List.getElem?_set]
    by_cases hk : i = k
    · subst hk; simp
    · simp [hk, Ne.symm hk]
  · simp only [hi, ite_false, List.getD_eq_getElem?_getD]
    rcases lt_trichotomy k i with h | h | h
    · have hne : k ≠ i := by omega
      by_cases hkl : k < l.length
      · simp [List.getElem?_append, hkl, hne]
      · have h1 : k - l.length < i - l.length := by omega
        simp [List.getElem?_append, hkl, hne, h1]
    · subst h
      simp [List.getElem?_append, hi]
    · have hne : k ≠ i := by omega
      have hkl : ¬ k < l.length := by omega
      have h1 : ¬ k - l.length < i - l.length := by omega
      obtain ⟨m, hm⟩ : ∃ m, k - l.length - (i - l.length) = m + 1 :=
        Nat.exists_eq_succ_of_ne_zero (by omega)
      simp [List.getElem?_append, hkl, hne, h1, hm]

/-! ## Expiry -/

theorem expired_iff (timeout : Nat) (stamp now : Nat) :
    expired timeout stamp now = true ↔ 0 < timeout ∧ stamp + timeout < now := by
  simp [expired]

/-- A handle stamped at `now` has not expired at `now`. -/
theorem not_expired_self (timeout : Nat) (now : Nat) : expired timeout now now = false := by
  simp [expired]

/-! ## Lock priority (R7) -/

/-- One iteration of the fold in `lockPriority`. -/
private def lockStep (cfg : Config) (st : State) (now : Nat)
    (acc : Nat) (hj : Handle × Nat) : Nat :=
  if lockActive cfg st now hj.2 then max acc hj.1.priority else acc

private theorem foldl_lockStep_spec (cfg : Config) (st : State) (now : Nat)
    (l : List (Handle × Nat)) (acc : Nat) :
    acc ≤ l.foldl (lockStep cfg st now) acc ∧
    (∀ hj ∈ l, lockActive cfg st now hj.2 = true →
      hj.1.priority ≤ l.foldl (lockStep cfg st now) acc) ∧
    (l.foldl (lockStep cfg st now) acc = acc ∨
      ∃ hj ∈ l, lockActive cfg st now hj.2 = true ∧
        l.foldl (lockStep cfg st now) acc = hj.1.priority) := by
  induction l generalizing acc with
  | nil => simp
  | cons x xs ih =>
    simp only [List.foldl_cons, List.mem_cons]
    obtain ⟨h1, h2, h3⟩ := ih (lockStep cfg st now acc x)
    by_cases hx : lockActive cfg st now x.2 = true
    · have hs : lockStep cfg st now acc x = max acc x.1.priority := by simp [lockStep, hx]
      rw [hs] at h1 h2 h3 ⊢
      refine ⟨by omega, ?_, ?_⟩
      · rintro hj (rfl | hmem) hact
        · omega
        · exact h2 hj hmem hact
      · rcases h3 with h3 | ⟨hj, hmem, hact, heq⟩
        · rcases Nat.le_total acc x.1.priority with hle | hle
          · right; exact ⟨x, Or.inl rfl, hx, by rw [h3]; omega⟩
          · left; rw [h3]; omega
        · right; exact ⟨hj, Or.inr hmem, hact, heq⟩
    · have hs : lockStep cfg st now acc x = acc := by simp [lockStep, hx]
      rw [hs] at h1 h2 h3 ⊢
      refine ⟨h1, ?_, ?_⟩
      · rintro hj (rfl | hmem) hact
        · exact absurd hact hx
        · exact h2 hj hmem hact
      · rcases h3 with h3 | ⟨hj, hmem, hact, heq⟩
        · left; exact h3
        · right; exact ⟨hj, Or.inr hmem, hact, heq⟩

private theorem lockPriority_eq (cfg : Config) (st : State) (now : Nat) :
    lockPriority cfg st now = cfg.locks.zipIdx.foldl (lockStep cfg st now) 0 := rfl

/-- Every locked lock has priority at most the lock priority. -/
theorem le_lockPriority (cfg : Config) (st : State) (now : Nat) (j : Nat) (hj : Handle)
    (hget : cfg.locks[j]? = some hj) (hact : lockActive cfg st now j = true) :
    hj.priority ≤ lockPriority cfg st now := by
  rw [lockPriority_eq]
  exact (foldl_lockStep_spec cfg st now _ 0).2.1 (hj, j)
    (List.mem_zipIdx_iff_getElem?.mpr hget) hact

/-- The lock priority is 0 or the priority of some locked lock. -/
theorem lockPriority_eq_zero_or (cfg : Config) (st : State) (now : Nat) :
    lockPriority cfg st now = 0 ∨
    ∃ j hj, cfg.locks[j]? = some hj ∧ lockActive cfg st now j = true ∧
      lockPriority cfg st now = hj.priority := by
  rw [lockPriority_eq]
  rcases (foldl_lockStep_spec cfg st now cfg.locks.zipIdx 0).2.2 with h | ⟨⟨hj, j⟩, hmem, hact, heq⟩
  · left; exact h
  · right; exact ⟨j, hj, List.mem_zipIdx_iff_getElem?.mp hmem, hact, heq⟩

/-- The lock priority is at most `p` iff every locked lock has priority at most `p`. -/
theorem lockPriority_le_iff (cfg : Config) (st : State) (now : Nat) (p : Nat) :
    lockPriority cfg st now ≤ p ↔
    ∀ j hj, cfg.locks[j]? = some hj → lockActive cfg st now j = true → hj.priority ≤ p := by
  constructor
  · intro h j hj hget hact
    exact le_trans (le_lockPriority cfg st now j hj hget hact) h
  · intro h
    rcases lockPriority_eq_zero_or cfg st now with h0 | ⟨j, hj, hget, hact, heq⟩
    · omega
    · rw [heq]; exact h j hj hget hact

/-! ## Selection (R8, R9) -/

private theorem pairwise_zipIdx {α : Type} (l : List α) (n : Nat) :
    (l.zipIdx n).Pairwise (fun a b => a.2 < b.2) := by
  induction l generalizing n with
  | nil => simp
  | cons x xs ih =>
    rw [List.zipIdx_cons, List.pairwise_cons]
    refine ⟨?_, ih (n + 1)⟩
    rintro ⟨y, m⟩ hmem
    have := (List.mem_zipIdx hmem).1
    simp only
    omega

/-- The selection loop returns either its starting value or the *first*
unmasked element of maximal priority. -/
private theorem foldl_selectStep_spec (cfg : Config) (st : State) (now : Nat) (lp : Nat)
    (l : List (Handle × Nat)) (hsorted : l.Pairwise (fun a b => a.2 < b.2))
    (acc : Nat × Option Nat) :
    acc.1 ≤ (l.foldl (selectStep cfg st now lp) acc).1 ∧
    (∀ hk ∈ l, unmasked cfg st now lp hk.2 = true →
      hk.1.priority ≤ (l.foldl (selectStep cfg st now lp) acc).1) ∧
    (l.foldl (selectStep cfg st now lp) acc = acc ∨
      ∃ hk ∈ l, unmasked cfg st now lp hk.2 = true ∧ acc.1 < hk.1.priority ∧
        l.foldl (selectStep cfg st now lp) acc = (hk.1.priority, some hk.2) ∧
        ∀ x ∈ l, unmasked cfg st now lp x.2 = true → x.2 < hk.2 →
          x.1.priority < hk.1.priority) := by
  induction l generalizing acc with
  | nil => simp
  | cons x xs ih =>
    simp only [List.foldl_cons, List.mem_cons]
    rw [List.pairwise_cons] at hsorted
    obtain ⟨hx_lt, hxs⟩ := hsorted
    obtain ⟨h1, h2, h3⟩ := ih hxs (selectStep cfg st now lp acc x)
    by_cases hx : unmasked cfg st now lp x.2 = true ∧ acc.1 < x.1.priority
    · have hs : selectStep cfg st now lp acc x = (x.1.priority, some x.2) := by
        simp [selectStep, hx.1, hx.2]
      rw [hs] at h1 h2 h3 ⊢
      simp only at h1
      refine ⟨by omega, ?_, ?_⟩
      · rintro hk (rfl | hmem) hun
        · exact h1
        · exact h2 hk hmem hun
      · right
        rcases h3 with h3 | ⟨hk, hmem, hun, hlt, heq, hfirst⟩
        · refine ⟨x, Or.inl rfl, hx.1, hx.2, h3, ?_⟩
          rintro y (rfl | hy) - hlt
          · omega
          · have := hx_lt y hy; omega
        · simp only at hlt
          refine ⟨hk, Or.inr hmem, hun, by omega, heq, ?_⟩
          rintro y (rfl | hy) hyun hylt
          · exact hlt
          · exact hfirst y hy hyun hylt
    · have hs : selectStep cfg st now lp acc x = acc := by
        unfold selectStep
        rw [ite_eq_right]
        simpa using hx
      rw [hs] at h1 h2 h3 ⊢
      refine ⟨h1, ?_, ?_⟩
      · rintro hk (rfl | hmem) hun
        · have : ¬ acc.1 < hk.1.priority := fun hlt => hx ⟨hun, hlt⟩
          omega
        · exact h2 hk hmem hun
      · rcases h3 with h3 | ⟨hk, hmem, hun, hlt, heq, hfirst⟩
        · left; exact h3
        · right
          refine ⟨hk, Or.inr hmem, hun, hlt, heq, ?_⟩
          rintro y (rfl | hy) hyun hylt
          · have : ¬ acc.1 < y.1.priority := fun h => hx ⟨hyun, h⟩
            omega
          · exact hfirst y hy hyun hylt

theorem unmasked_iff (cfg : Config) (st : State) (now : Nat) (lp : Nat) (k : Nat) :
    unmasked cfg st now lp k = true ↔
    ∃ hk, cfg.inputs[k]? = some hk ∧ inputLive cfg st now k = true ∧ lp ≤ hk.priority := by
  unfold unmasked
  cases h : cfg.inputs[k]? with
  | none => simp
  | some hk => simp

/-- **Selection.** Input `i` is selected iff it is configured with a positive
priority, is live, is not outranked by the lock priority, and every other live
input either has strictly lower priority or has the same priority and comes
later in the list (R9, R10). -/
theorem select_eq_some_iff (cfg : Config) (st : State) (now : Nat) (i : Nat) :
    select cfg st now = some i ↔
    ∃ hi, cfg.inputs[i]? = some hi ∧ 0 < hi.priority ∧ inputLive cfg st now i = true ∧
      lockPriority cfg st now ≤ hi.priority ∧
      ∀ k hk, cfg.inputs[k]? = some hk → k ≠ i → inputLive cfg st now k = true →
        hk.priority < hi.priority ∨ (hk.priority = hi.priority ∧ i < k) := by
  have hsel : select cfg st now =
      (cfg.inputs.zipIdx.foldl (selectStep cfg st now (lockPriority cfg st now)) (0, none)).2 := rfl
  obtain ⟨-, h2, h3⟩ := foldl_selectStep_spec cfg st now (lockPriority cfg st now)
    cfg.inputs.zipIdx (pairwise_zipIdx ..) (0, none)
  constructor
  · intro h
    rw [hsel] at h
    rcases h3 with h3 | ⟨⟨hi, i'⟩, hmem, hun, hlt, heq, hfirst⟩
    · rw [h3] at h; simp at h
    · rw [heq] at h
      simp only [Option.some.injEq] at h
      subst h
      have hgi := List.mem_zipIdx_iff_getElem?.mp hmem
      obtain ⟨hi', hgi', hlive, hlp⟩ := (unmasked_iff ..).mp hun
      rw [hgi] at hgi'; cases hgi'
      refine ⟨hi, hgi, hlt, hlive, hlp, ?_⟩
      intro k hk hgk hne hklive
      by_cases hmask : lockPriority cfg st now ≤ hk.priority
      · have hkun := (unmasked_iff ..).mpr ⟨hk, hgk, hklive, hmask⟩
        have hkmem : (hk, k) ∈ cfg.inputs.zipIdx := List.mem_zipIdx_iff_getElem?.mpr hgk
        have hle := h2 (hk, k) hkmem hkun
        rw [heq] at hle
        simp only at hle
        by_cases hki : k < i'
        · left; exact hfirst (hk, k) hkmem hkun hki
        · omega
      · left; exact lt_of_lt_of_le (not_le.mp hmask) hlp
  · rintro ⟨hi, hgi, hpos, hlive, hlp, hothers⟩
    have hun : unmasked cfg st now (lockPriority cfg st now) i = true :=
      (unmasked_iff ..).mpr ⟨hi, hgi, hlive, hlp⟩
    have himem : (hi, i) ∈ cfg.inputs.zipIdx := List.mem_zipIdx_iff_getElem?.mpr hgi
    have hle := h2 (hi, i) himem hun
    rw [hsel]
    rcases h3 with h3 | ⟨⟨hk, k⟩, hmem, hunk, -, heq, hfirst⟩
    · rw [h3] at hle; simp only at hle; omega
    · rw [heq]
      rw [heq] at hle
      simp only at hle ⊢
      by_contra hne
      have hne' : k ≠ i := fun h => hne (by rw [h])
      have hgk := List.mem_zipIdx_iff_getElem?.mp hmem
      obtain ⟨hk', hgk', hklive, -⟩ := (unmasked_iff ..).mp hunk
      rw [hgk] at hgk'; cases hgk'
      rcases hothers k hk hgk hne' hklive with h | ⟨heq', hik⟩
      · omega
      · have := hfirst (hi, i) himem hun hik
        simp only at this
        omega

end ROSlean.Mux
