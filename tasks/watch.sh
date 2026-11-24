#!/usr/bin/env bash
# Is a task session stuck? For each git worktree of this repository that is on
# a task branch (claude/*), prints how long ago a file last changed and what
# is running there, and a verdict:
#
#   active   a file changed in the last STALE minutes, or processes are busy
#   waiting  nothing changed, but processes of the worktree exist (a long test
#            that waits in real time looks like this; see the process list)
#   STUCK    nothing changed for STALE minutes and nothing is running
#   done     the branch's head is pushed and the tree is clean
#
#   tasks/watch.sh [STALE minutes, default 20]
#
# Exit status 1 if any worktree is STUCK. Used by the orchestrator's /loop
# (tasks/PLAN.md, "Watching the tasks").
set -uo pipefail
stale="${1:-20}"
export GIT_OPTIONAL_LOCKS=0   # git status below must not touch the index: its age is evidence
root="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
main="$(git -C "$root" worktree list --porcelain | awk '/^worktree /{print $2; exit}')"
status=0
now=$(date +%s)
mtime() { stat -f %m "$1" 2>/dev/null || stat -c %Y "$1" 2>/dev/null; }

while read -r wt; do
  [ "$wt" = "$main" ] && continue
  branch="$(git -C "$wt" branch --show-current 2>/dev/null)"
  case "$branch" in claude/*) ;; *) continue ;; esac
  # A worktree whose head is already in the integration branch and whose git
  # state has not moved for an hour belongs to a finished, merged task: skip
  # it, so that the scan below stays fast when many worktrees exist.
  gd="$(git -C "$wt" rev-parse --absolute-git-dir)"
  if git -C "$wt" merge-base --is-ancestor HEAD origin/collision-monitor 2>/dev/null \
     && [ $(( (now - $(mtime "$gd/logs/HEAD")) / 60 )) -ge 60 ] \
     && ! ps -eo command= | grep -F "$wt" | grep -qv -e 'grep -F' -e 'tasks/watch.sh'; then
    continue
  fi
  # Newest file: sources, colcon logs, the project's own Lean build, git's state.
  newest="$( { find "$wt" -type f \
        -not -path '*/.pixi/*' -not -path '*/.lake/packages/*' -not -path '*/node_modules/*' \
        -mmin -1440 2>/dev/null || true; \
      ls -d "$(git -C "$wt" rev-parse --absolute-git-dir)"/{HEAD,logs/HEAD} 2>/dev/null; } \
    | while read -r f; do echo "$(mtime "$f") $f"; done | sort -n | tail -1)"
  [ -z "$newest" ] && continue          # untouched for a day: an old worktree
  age=$(( (now - ${newest%% *}) / 60 ))
  procs="$(ps -eo pid=,etime=,pcpu=,command= | grep -F "$wt" | grep -v -e 'grep -F' -e 'tasks/watch.sh')"
  nproc=$(printf '%s' "$procs" | grep -c . || true)
  cpu=$(printf '%s\n' "$procs" | awk '{s+=$3} END {printf "%.0f", s}')
  head="$(git -C "$wt" rev-parse HEAD)"
  remote="$(git -C "$wt" rev-parse -q --verify "origin/$branch" 2>/dev/null || true)"
  dirty=$(git -C "$wt" status --porcelain | grep -c . || true)
  if [ "$head" = "$remote" ] && [ "$dirty" = 0 ] && [ "$nproc" = 0 ]; then verdict=done
  elif [ "$age" -lt "$stale" ] || [ "${cpu:-0}" -ge 5 ]; then verdict=active
  elif [ "$nproc" -gt 0 ]; then verdict=waiting
  else verdict=STUCK; status=1
  fi
  printf '%-8s %-28s last change %3d min ago  %2d processes (%s%% cpu)  %d uncommitted  %s\n' \
    "$verdict" "$branch" "$age" "$nproc" "${cpu:-0}" "$dirty" "$(git -C "$wt" log -1 --format='%h %ar')"
  printf '         newest: %s\n' "${newest#* }" | sed "s|$wt/||"
  printf '%s\n' "$procs" | awk 'NF {printf "         pid %s, running %s, %s%% cpu: ", $1, $2, $3; for (i=4;i<=NF&&i<=7;i++) printf "%s ", $i; print ""}' \
    | sed "s|$wt/||g" | cut -c1-200 | head -6
done < <(git -C "$root" worktree list --porcelain | awk '/^worktree /{print $2}')
exit $status
