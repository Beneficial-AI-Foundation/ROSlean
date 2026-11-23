# T11 — Website, README and trusted-base statement

Branch: `claude/cm-writeup`. Needs: T09 merged (and T10 if it was done).
Read [README.md](README.md) first.

## Read

Everything under `docs/collision_monitor/`, `docs/REVIEWS.md`,
`docs/PLAN.md`, `README.md`, and the site (`site/index.html`,
`site/details.html`, `site/style.css`; deployed by `.github/workflows/pages.yml`
on pushes to `main`, so the site changes go live only when the user merges `collision-monitor` into `main`).

## Do

1. **A website page on what is missing** (`site/gaps.html`, linked from the
   other pages, in the site's existing style). It must be precise enough that
   a robotics engineer can decide whether the node fits their robot, and a
   contributor can pick up a gap. Built from SPEC_NAV2.md's section "What
   still separates this from Nav2's default configuration", SPEC.md's
   differences and trusted list, THEOREMS.md's "What is not proved",
   COMPARISON.md and the open findings in REVIEWS.md. For each gap:
   - what it is, in one plain sentence, and what upstream does there (with
     the source file);
   - whether Nav2's default configuration uses it;
   - what the Lean node does today when asked for it (refuses, ignores,
     differs: the exact message or behaviour);
   - what closing it takes: the work items, what would be proved, what would
     stay trusted, and a size relative to work already done in this
     repository (for example "about the size of the stop and slowdown core").
   Group the gaps: actions (`approach`, `limit`), zone shapes that change,
   sensors and sources, proofs not done (the topological half of the polygon
   margin argument, the float multiplication, the transform code, the IEEE
   reading of bit patterns), platforms never tested, open decisions and the
   behaviour each default implies (above all the stop that is never
   published after `stop_pub_timeout`). State first, in three sentences,
   what the node is and is not: a proved gate for stop and slowdown zones
   that Nav2 can launch, not a drop-in for Nav2's default `approach`
   configuration. Check every statement against the repository; a claim
   that cannot be traced to a file is removed.
2. **The site's existing pages**: add the collision monitor beside
   twist_mux (what it does, what is proved, the demo, the comparison
   numbers), without overstating: the claim is about commands, not physics.
3. **README.md**: the layout table, quick start, the collision monitor's
   "What is proved" and "What is not proved, and is trusted or only tested",
   how to run it under Nav2, links to the documents.
4. **`docs/PLAN.md`**: milestones CM1 onward with their results, the "Not
   done" list, and the lessons from this node (collect the "Friction"
   sections of `tasks/reports/`).
5. Update `tasks/PLAN.md`: status of every task, decisions still open.

## May change

`site/`, `README.md`, `docs/PLAN.md`, `tasks/PLAN.md`, small corrections in
`docs/`.
