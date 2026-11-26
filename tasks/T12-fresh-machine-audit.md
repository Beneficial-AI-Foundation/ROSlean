# T12 — Fresh-machine audit

Branch: `claude/cm-audit`. Needs: T11 merged. Do not run `scripts/cloud_setup.sh`
and do not read `tasks/README.md`'s environment section before step 1: the
point is to find out whether the README alone is enough.

## Do

1. Starting from a fresh machine and `README.md` only, follow it literally:
   install, build, test, prove, run each node, run what it says about Nav2.
   Record every place where an instruction was missing, wrong, ambiguous or
   failed, with the output.
2. Check every number and claim on the website pages (`site/`) and in the
   README against the repository: theorem names exist and say what the text
   says (`#print axioms` on each named theorem), test counts match a run,
   benchmark numbers match `bench/results/` and COMPARISON.md, each gap on
   `site/gaps.html` matches the code's actual behaviour (try each refused
   configuration and compare the message).
3. Search the Lean sources for `sorry`, `native_decide`, `axiom`, `unsafe`,
   `implemented_by` and `@[extern]`, and say what each hit is.

## Deliver

`docs/AUDIT.md`: what worked, what did not, each discrepancy with its
evidence. Fix plain errors in the README and the site in the same branch
(wrong command, wrong number, dead link) and list them; leave anything that
is a matter of substance as a finding.

## May change

`docs/AUDIT.md`, and corrections of plain errors in `README.md` and `site/`.
