# T08 — Independent review of the Nav2 extension

Branch: `claude/cm-nav2-review`. Needs: T06 and T07 merged.
Read [README.md](README.md) first.

You did not write any of this. Your job is to find what is wrong, not to
confirm that it is right. Read the earlier reviews in `docs/REVIEWS.md` for
the form and for the kind of defect that matters.

## Do

1. Run the Lean node through every scenario of T07 and the extended random
   comparison beside upstream (several hundred seeds). Classify each
   difference: a numbered D or ND, an unrecorded difference, a defect of the
   node or model, or a defect of the harness. A case where upstream stops or
   slows and the Lean node does not comes first.
2. SPEC_NAV2.md against the upstream source, rule by rule.
3. The model's new events against SPEC_NAV2.md; the new theorems: true of
   what the node calls, non-vacuous, as strong as THEOREMS.md says.
4. The node: any decision left in C++; any path where a command is published
   without passing through Lean; the lifecycle transitions under a real
   `nav2_lifecycle_manager`; undefined behaviour for any message or
   transform content; the trusted transform code (sign, axis order, time of
   the lookup, precision) against upstream on many random transforms.
5. A real configuration: `nav2_bringup`'s parameters with the single edit,
   the node started by Nav2's own launch file with the package name changed.
   If a full Nav2 bring-up cannot run where you are, say so.
6. Tests that cannot fail, and behaviour no test covers.

## Deliver

A section "CM5 — Nav2 extension" appended to `docs/REVIEWS.md`: a one-line
verdict and a table of findings by severity, each with its smallest input and
the outcome "open". Scripts under `review/collision_monitor/nav2/`. The
comparison results added to `docs/collision_monitor/COMPARISON.md`. Turn on
the Lean-node registrations of T07's scenarios that pass; leave the failing
ones as expected failures naming the finding. Fix nothing in the node, model
or SPEC.

## May change

`docs/REVIEWS.md`, `review/`, COMPARISON.md, harness registrations.
